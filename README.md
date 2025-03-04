## Building a camera driver from scratch - PS3 Eye Camera on Mac Silicon

I wrote a blog post about the project: [building a camera driver from scratch](https://cetusian.com/projects/ps3eye_driver).

![PS3 Eye Camera](ps3eye_camera_1.png)

### Steps to run the project

#### Clone the repo
```bash
 git clone https://github.com/madalintat/PS3Eye-Driver-MacOS-Silicon.git
```

#### Enter the repo
```bash
cd PS3Eye-Driver-MacOS-Silicon
```

#### Compile the driver
```bash
gcc -o ps3eye_test ps3eye_test.c -I/opt/homebrew/Cellar/libusb/1.0.27/include/libusb-1.0 -L/opt/homebrew/Cellar/libusb/1.0.27/lib -lusb-1.0
```

#### Run the driver
```bash
./ps3eye_test
```
---

##### Success: Prints device info, "Camera initialized...", and "Camera test successful!" then exits cleanly.

##### Failure: Prints an error message at any step (e.g., device not found, permission denied) and exits with return 1.

---

> Disclaimer: the driver creates the connection between the PS3 Eye camera and the Macbook device. Capturing video is a different job, but it's part of my upcoming project. Follow my [website](https://cetusian.com) for updates.
