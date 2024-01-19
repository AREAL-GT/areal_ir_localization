
#include "tracker_structs.hpp"
#include <string>
#include <map>

/***
 * A beacon is a circular doublely linked list of nodes, with each node 
 * containing a contour
 * We are assuming every countour has been malloced and will be freed elsewhere  
 * Thus we only need to worry about the memory of the nodes themselves
*/

class Beacon{

public:
    Beacon(){
        start_ = nullptr;
        size_ = 0;
        min_area_ = 0;
        max_area_ = 10000000; // Arbitrary large number
        avg_area_ = 0.0;
        threshold_area_ = 0;
        edge_index_ = 0;
        position_ = {0,0};
        radius_ = 0.0;
        message_ = 0;
        bits_per_message_ = 8;
        camera_frame_rate_ = 50;
        message_baud_rate_ = 5;
        frames_per_message_ = 
            (camera_frame_rate_ / message_baud_rate_) * bits_per_message_;
        valid_ = false;
        lookup_table_ptr_ = nullptr;
        full_string_ = "";
    };

    Beacon(int bits_per_message, int camera_frame_rate, int message_baud_rate, 
            std::map<int, int>* lookup_table_ptr){
        start_ = nullptr;
        size_ = 0;
        min_area_ = 0;
        max_area_ = 10000000; // Arbitrary large number
        avg_area_ = 0.0;
        threshold_area_ = 0;
        edge_index_ = 0;
        position_ = {0,0};
        radius_ = 0.0;
        message_ = 0;
        bits_per_message_ = bits_per_message;
        camera_frame_rate_ = camera_frame_rate;
        message_baud_rate_ = message_baud_rate;
        frames_per_message_ = 
            (camera_frame_rate_ / message_baud_rate_) * bits_per_message_;
        valid_ = false;
        lookup_table_ptr_ = lookup_table_ptr;
        full_string_ = "";
    };
    
    ~Beacon(){

        // If list is empty, return
        if(start_ == nullptr){
            return;
        }

        // If list is not empty, free all nodes
        Node<Contour>* current = start_;
        Node<Contour>* next = start_->next;
        while(next != start_){
            free(current);
            current = next;
            next = next->next;
        }
        free(current);
    };

    void set_lookup_table(std::map<int, int>* lookup_table_ptr){
        lookup_table_ptr_ = lookup_table_ptr;
    };

    // Getters
    Node<Contour>* get_start(){return start_;};
    int get_size(){return size_;};
    float get_min_area(){return min_area_;};
    float get_max_area(){return max_area_;};
    float get_avg_area(){return avg_area_;};
    int get_edge_index(){return edge_index_;};
    Vector2 get_position(){return position_;};
    float get_radius(){return radius_;};
    int get_message(){return message_;};
    int get_id(){return id_;};
    bool is_valid(){return valid_;};
    std::string get_message_string(){
        std::string message_string = "";
        for(int i = 0; i < bits_per_message_; i++){
            message_string += 
                std::to_string((message_ >> (bits_per_message_ - i - 1)) & 1);
        }
        return message_string;
    };
    std::string get_full_string(){
        return full_string_;
    }

    // Method to get the full set of bits from all frames as string
    std::string get_bits_str(){
        // Returns bits start->end left->right in a string
        // Start is newest frame, end is oldest frame, new->old

        Node<Contour>* curr = start_;
        std::string bit_text = "";

        // Loop through all bits in beacon
        for(int i = 0; i < size_; i++){
            
            // Add bit value to string
            bit_text += std::to_string(curr->data.bit);

            // Iterate to next node
            curr = curr->next;

        }

        // Return completed string
        return bit_text;

    }
    
    // Add a node to the end of the list
    void add(Contour* contour){

        // Check if countour is null
        if(contour == nullptr){
            return;
        }

        // Create new node
        Node<Contour>* new_node = new Node<Contour>;
        new_node->data = *contour;

        // Check if list is empty.  If so, set start to new node
        if(start_ == nullptr){
            start_ = new_node;
            start_->next = start_;
            start_->prev = start_;
            size_++;

            // Set position to start position
            position_ = start_->data.position;

            // Set radius to that of most recent beacon
            radius_ = start_->data.radius;
            
            // Set min and max area to contour area
            min_area_ = contour->area;
            max_area_ = contour->area;

            // Set avg area to current area
            avg_area_ = contour->area;
            return;
        }

        // Compute total area
        float total_area = avg_area_*size_;

        // If list is not empty, add new node to end of list
        Node<Contour>* end = start_->prev;
        end->next = new_node;
        new_node->prev = end;
        new_node->next = start_;
        start_->prev = new_node;
        size_++;

        // Update min and max area
        if(contour->area < min_area_){
            min_area_ = contour->area;
        }
        if(contour->area > max_area_){
            max_area_ = contour->area;
        }

        // Update average area
        total_area += contour->area;
        avg_area_ = total_area/size_;
        
        return;
    };

    /***
     * This method iterates over each node, performs smoothing, data conversion, 
     * edge detection, and message creation
    */
    std::string process_beacon(bool debug_print = false){

        std::string debug_string = "";
        std::string size_str = "Sizes:\n";
        std::string bit_str = "Bits: ";

        // If size is less than frames per message, return
        if(size_ < frames_per_message_){
            valid_ = false;
            return debug_string;
        }
        valid_ = true;

        // Determine threshold area
        threshold_area_ = (min_area_ + max_area_) / 2;

        // Iterate over the first two nodes and determine their bit values
        Node<Contour>* curr = start_;
        curr->data.bit = curr->data.area > threshold_area_ ? 1 : 0;
        curr = curr->next;
        curr->data.bit = curr->data.area > threshold_area_ ? 1 : 0;
        curr = curr->next;

        /***
         * Iterate over the rest of the nodes and determine their bit values
         * Perform both smoothing and edge detection
         * (11/8/23) - Bit smoothing turned off for now b/c not needed
        */
        edge_index_ = -1;
        for(int i = 2; i < size_ + 2; i++){

            // Determine bit value only if bit is -1.  
            //-1 indicates that the bit has not been determined yet
            if(curr->data.bit == -1){
                curr->data.bit = curr->data.area > threshold_area_ ? 1 : 0;
            }

            /* Commented out 11/8/23
            // Perform smoothing of previous bit by comparing curr to prev.prev 
            // and curr to prev
            if(curr->data.bit == curr->prev->prev->data.bit && 
                curr->data.bit != curr->prev->data.bit){
                curr->prev->data.bit = curr->data.bit;
            }
            */

            // Perform edge detection
            if((curr->data.bit != curr->prev->data.bit) & (edge_index_ == -1)){
                edge_index_ = i;
            }

            // Iterate to next node
            curr = curr->next;
        }

        // Set the edge index to the edge nearest to the start
        int frames_per_bit = camera_frame_rate_ / message_baud_rate_;

        // Integer division rounds down.  
        // Essentially, this is edge_index_ % frames_per_bit
        edge_index_ -= (edge_index_ / frames_per_bit) * frames_per_bit; 
        
        // Point curr to edge
        curr = start_;
        for(int i = 0; i < edge_index_; i++){
            curr = curr->next;
        }

        // Iterate over the rest of the nodes and create the message
        message_ = 0;
        
        for(int i = 0; i < bits_per_message_; i++){

            int sum = 0;

            for (int ii = 0; ii < frames_per_bit; ii++){
                
                sum += curr->data.bit;

                // Add data and sizes to debug strings
                if (debug_print){
                    size_str += std::to_string((int)curr->data.area) + ",";
                    bit_str += std::to_string(curr->data.bit);
                }

                curr = curr->next;

            }

            // Debug string editing per-bit
            if (debug_print){
                size_str.pop_back(); // Remove trailing comma
                size_str += "\n"; // Sizes for next bit on new line
                bit_str += " ";
            }

            // If sum is greater than half of frames_per_bit, set bit to 1, 
            // else set bit to 0
            message_ = message_ << 1;
            if (sum > (frames_per_bit / 2)){
                message_ += 1; 
            } 
        }

        // Set id based on lookup table
        id_ = (*lookup_table_ptr_)[message_];

        if (debug_print){
            bit_str.pop_back(); // Remove trailing space
            bit_str += "\n";
            debug_string = size_str + bit_str;
        }

        return debug_string;
    };


private:
    Node<Contour>* start_;
    int size_;
    float min_area_, max_area_, threshold_area_, avg_area_;
    float radius_;
    int edge_index_;
    Vector2 position_;
    int message_;
    int bits_per_message_;
    int camera_frame_rate_;
    int message_baud_rate_;
    int frames_per_message_;
    int id_;
    bool valid_;
    std::string full_string_;
    std::map<int, int>* lookup_table_ptr_;
};