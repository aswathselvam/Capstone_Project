#include "main.h"
/*
CameraDriver::CameraDriver()
{

	ros::NodeHandle n("my_camera_publisher");

	n.param("camera_index", id_camera, -1);
	n.param("show", show, false);
	n.param("fps", fps, 30);

	cvi.header.frame_id = "image";
	cvi.encoding = sensor_msgs::image_encodings::BGR8;



	it = new image_transport::ImageTransport(n);

	pub = it->advertise("/image_raw", 1);


	if (id_camera == -1)
	{
		ROS_WARN("camera's id has not recived");
		ROS_WARN("I will open  every camera that I find :P ");
	}

	input_video.open(id_camera);
	input_video.set(CV_CAP_PROP_FPS, fps);

	if (!input_video.isOpened())
	{
		ROS_ERROR("Couldn't Open The Camera !");
		ROS_ERROR("Babay :(");
		ros::shutdown();
	}


	ros::Rate loop_rate(fps);

	while (ros::ok())
	{

		input_video.read(frame);
		cvi.image = frame;
		cvi.header.stamp = ros::Time::now();
		pub.publish(cvi.toImageMsg());

		if (show)
		{
			imshow("imgout.jpg", frame);
			if (waitKey(1) > 0);
		}

		loop_rate.sleep();

	}



}
*/
int main() {


}