#include "simple_screen_listener.h"

namespace deepf1
{

	simple_screen_listener::simple_screen_listener(boost::shared_ptr<const boost::timer::cpu_timer>& timer,
		cv::Rect2d capture_area,
		unsigned int monitor_number,
		unsigned int length)
	{
		this->timer = timer;
		this->length = length;
		dataz = new timestamped_image_data_t[length];
		svc.reset(new screen_video_capture(capture_area, monitor_number));
		this->capture_area = svc->capture_area();
		init_images(this->capture_area.height, this->capture_area.width);
	}
	simple_screen_listener::~simple_screen_listener()
	{
		for (unsigned int i = 0; i < length; i++) {
			delete dataz[i].image;
		}
		delete[] dataz;
	}
	timestamped_image_data_t* simple_screen_listener::get_data()
	{
		return dataz;
	}
	void simple_screen_listener::init_images(int num_rows, int num_columns)
	{
		for (unsigned int i = 0; i < length; i++) {
			dataz[i].image = new cv::Mat(num_rows, num_columns, CV_8UC4);
		}
	}
	void simple_screen_listener::listen()
	{
		for (unsigned int i = 0; i < length; i++)
		{
			printf("Reading image\n");
			dataz[i].timestamp = timer->elapsed();
			svc->read(dataz[i].image);
		}
	}

}