**Package:** ROS msgs package, `common`, available at https://github.com/SmileJayjhx/common

**Steps to Reproduce:**

1. Download `common` and `ac_node`.
2. Navigate to the `common` directory, run `bloom-generate rosdebian`, followed by `fakeroot debian/rules binary`.
3. Navigate to the `ac_node` directory, run `bloom-generate rosdebian`, followed by `fakeroot debian/rules binary`.
4. Install the packages using `sudo dpkg -i ../*.deb`.
5. Launch the node with `roslaunch ac_node start_ac_node.launch`.
6. Publish to the topic using `rostopic pub -r 10 /peripheral_devs_state common/peripheral_uart`.
7. Observe the main function restarts.

![Screenshot from 2024-03-12 15-07-23](https://gitee.com/smilejay/ubuntu-picture-bed/raw/master/img/20240312150818.png)

I add `[ERROR] [1710227236.688463076]: getting into main` at the beginning of main function.
