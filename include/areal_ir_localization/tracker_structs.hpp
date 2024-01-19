
typedef struct Vector2
{
  int x;
  int y;
} Vector2;

typedef struct Contour
{
  float area;
  float radius;
  Vector2 position;
  int bit;
} Contour;

typedef struct Frame
{
  Contour *contour_list;
  int num_contours;
  int *association_to_prev;
} Frame;

template <typename T>
struct Node
{
  T data;
  Node<T> *next;
  Node<T> *prev;
};

typedef struct BeaconPos
{
  int pcbPort;
  float xPos;
  float yPos;
  float zPos;
} BeaconPos;
