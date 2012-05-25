#ifndef EXECUTIVE_H_
#define EXECUTIVE_H_

namespace navi_executive {

struct Waypoint {
    double lat, lon;
};

class Executive {
public:
    explicit Executive(void);
    ~Executive(void);

private:
    std::vector<std::vector<Waypoint> > waypoints_;
};

};

#endif
