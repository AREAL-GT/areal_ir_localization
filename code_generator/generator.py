#Code generator for areal_ir_localization.  Generates a codebook and lookup table for a given number of bits per message.

import csv

BITS_PER_MESSAGE = 12

is_noise = True
see_binary = False #if true, will print binary values into the csv files instead of decimal values

lookup_table = []
used_ids = []
code_book = []
values = []
keys = []

# iterate through all codes
for code in range(0, 2**BITS_PER_MESSAGE - 1):
    in_c = False 
    id_list = []

    for i in range(0, BITS_PER_MESSAGE): #rotate code by i bits. 
        #rotate code by i bits
        #       |          shift left by i bits        | Stick "lost" MSBs on front by shift right BITS_PER_MESSAGE - i bits | 
        curr = ((((code & (2**BITS_PER_MESSAGE - 1)) << i)  | ((code & (2**BITS_PER_MESSAGE - 1)) >> (BITS_PER_MESSAGE - i))) 
            & (2**BITS_PER_MESSAGE - 1)) #Mask out only the first BITS_PER_MESSAGE bits  
        
        #add curr to id_list.  We are fine if curr is already in id_list because this means that a noisy version this code shifted still returns the same id
        if curr in id_list:
            id_list.append(curr)

        #noisify 
        noise = []
        if is_noise:
            
            for ii in range(0, BITS_PER_MESSAGE):
                #perform bit flips on curr.  Iterate through each bit and flip it
                noise.append(curr ^ (1 << ii))

                #perform bit deletions on curr.  Iterate through each bit and delete it using a mask
                noise.append(curr & ~(1 << ii))  
            
                #perform bit insertions of 1 on curr.  Shift curr right by ii bits, insert 1, then shift back and OR with mask on curr
                temp = ((curr >> ii) << 1) | 1 
                temp = (temp << ii) | (curr & (2**ii - 1))
                noise.append(temp & (2**BITS_PER_MESSAGE - 1))

                #perform bit insertions of 0 on curr. Shift curr right by ii bits, insert 0, then shift back and OR with mask on curr
                temp = (curr >> ii) << 1
                temp = (temp << ii) | (curr & (2**ii - 1))
                noise.append(temp & (2**BITS_PER_MESSAGE - 1))
            
            #remove duplicates from noise
            noise = list(dict.fromkeys(noise))

        #append all noise to id_list
        for n in noise:
            id_list.append(n)
            in_c = in_c or (n in used_ids)
    
    #if code is not in codebook and not in_c, add it to codebook and add all ids to lookup_table
    if not in_c:
        code_book.append(code)
        # remove duplicates from id_list.  We can do this because we know that duplicates here are the same code shifted by different amounts with noise
        id_list = list(dict.fromkeys(id_list))
        for id in id_list:
            if see_binary:
                lookup_table.append([bin(id), code])
            else:
                # lookup_table.append([id, code])
                keys.append(code)
                values.append(id)
            used_ids.append(id)

#write codebook to csv
# field names 
codebook_fields = ['Code'] 
lookup_table_fields = ['Code', 'ID'] 

#append keys and values to lookup_table
lookup_table.append(keys)
lookup_table.append(values)

filename = "codes/codebook_" + str(BITS_PER_MESSAGE) +".csv"
# writing to csv file 
with open(filename, 'w') as csvfile: 
    # creating a csv writer object 
    csvwriter = csv.writer(csvfile) 
        
    # writing the fields 
    csvwriter.writerow(codebook_fields) 
        
    # writing the data rows 
    for code in code_book:
        if see_binary:
            csvwriter.writerow([bin(code)])
        else:
            csvwriter.writerow([code])
    # csvwriter.writerows(codebook_data)

filename = "codes/lookup_table_" + str(BITS_PER_MESSAGE) +".csv"
# writing to csv file 
with open(filename, 'w') as csvfile: 
    # creating a csv writer object 
    csvwriter = csv.writer(csvfile) 
        
    # writing the fields 
    csvwriter.writerow(lookup_table_fields) 
        
    # writing the data rows 
    csvwriter.writerows(lookup_table)