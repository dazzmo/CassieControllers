#include "controller/os_ctrl.h"

int main(void) {
    CassieOSController* c = new CassieOSController();

    c->qpos() << 0, 0, 1.0059301, 1, 0, 0, 0,
        0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681,
        -0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681;

    c->HeelSpringDeflection();
    return 0;
}
