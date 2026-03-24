<launch>
    <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="yuyv" />
    </node>

    <node name="lane_detector" pkg="your_package_name" type="lane_node.py" />
    <node name="sign_detector" pkg="your_package_name" type="sign_node.py" />

    <node name="decision_center" pkg="your_package_name" type="decision_node.py" output="screen" />
</launch>