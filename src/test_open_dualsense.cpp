#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#define JOY_DEV "/dev/input/js0"

int main(int argc, char *argv[]) {

    int joy_fd(-1), num_of_axis(0), num_of_buttons(0);
    char name_of_joystick[80];
    std::vector<char> joy_button;
    std::vector<int> joy_axis;

    if ((joy_fd = open(JOY_DEV, O_RDONLY)) < 0) {
        std::cerr << "Failed to open " << JOY_DEV << std::endl;
        return -1;
    }
    std::cout << "FD : " << joy_fd << std::endl;

    ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
    ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
    ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);
    std::exit(0);
    return 0;
}
