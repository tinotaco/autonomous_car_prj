
typedef struct {
    float x;
    float y;
} coordinates;

typedef struct {
    coordinates rel_coord;
} pose_laserpoint;

typedef struct {
    float x; //in meters
    float y; //in meters
    float yaw; //in Radians
} twodpose_t;

typedef struct {
    int cell_column;
    int cell_row;
} gridcell;

typedef struct {
    gridcell cell;
    bool occupied;
} occcell_update;

typedef struct {
    float P00;
    float P10;
    float P01;
    float P11;
    float x_g0;
    float x_g1;
    float y_g0;
    float y_g1;
} gridedges;