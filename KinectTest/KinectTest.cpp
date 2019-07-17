

#include "stdafx.h"
#include "k4a/k4a.h"
#include <k4abt.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \


double pmax = 0, pmin = 1e18;

void show_image_index(k4a_image_t image) {
	if (image != NULL) {
		printf("| Depth16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(image),
			k4a_image_get_width_pixels(image),
			k4a_image_get_stride_bytes(image));
	}
	int width = k4a_image_get_width_pixels(image);
	int height = k4a_image_get_height_pixels(image);
	int stride = k4a_image_get_stride_bytes(image) / sizeof(uint8_t);
	uint8_t* image_buffer = k4a_image_get_buffer(image);
	cv::Mat m(height, width, CV_8U);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			m.at<uchar>(i, j) = image_buffer[i*stride + j];
		}
	}
	cv::imshow("index", m);
	cv::waitKey(0);
}

void show_image_depth(k4a_image_t image) {
	if (image != NULL) {
		printf("| Depth16 res:%4dx%4d stride:%5d\n",
			k4a_image_get_height_pixels(image),
			k4a_image_get_width_pixels(image),
			k4a_image_get_stride_bytes(image));
	}
	int width = k4a_image_get_width_pixels(image);
	int height = k4a_image_get_height_pixels(image);
	int stride = k4a_image_get_stride_bytes(image) / sizeof(uint8_t);
	uint8_t* image_buffer = k4a_image_get_buffer(image);
	cv::Mat m(height, width, CV_8U);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width * 2; j += 2) {
			double t = ((uint16_t)(image_buffer[i*stride + j])) + (double)image_buffer[i*stride + j + 1] * (1 << 8);
			pmax = std::max(pmax, t);
			pmin = std::min(pmin, t);
		}
	}
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width * 2; j += 2) {
			double t = ((uint16_t)(image_buffer[i*stride + j])) + (double)image_buffer[i*stride + j + 1] * (1 << 8);
			m.at<uchar>(i, j / 2) = 256 - (double)(t - pmin) / (pmax - pmin) * 256;
		}
	}
	cv::imshow("depth", m);
	cv::waitKey(0);
}

int kbt()
{
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

	k4a_device_t device;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
	VERIFY(k4a_device_start_cameras(device, &device_config), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration),
		"Get depth camera calibration failed!");

	k4abt_tracker_t tracker = NULL;
	VERIFY(k4abt_tracker_create(&sensor_calibration, &tracker), "Body tracker initialization failed!");

	int frame_count = 0;
	do
	{
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
		k4a_image_t image = k4a_capture_get_depth_image(sensor_capture);
		show_image_depth(image);
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			frame_count++;
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
			k4a_capture_release(sensor_capture);
			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!\n");
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
			{
				printf("Error! Add capture to tracker process queue failed!\n");
				break;
			}

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%zu bodies are detected!\n", num_bodies);
				k4a_image_t image = k4abt_frame_get_body_index_map(body_frame);
				show_image_index(image);

				k4abt_frame_release(body_frame);
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!\n");
				break;
			}
		}
		else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else
		{
			printf("Get depth capture returned error: %d\n", get_capture_result);
			break;
		}

	} while (1);

	printf("Finished body tracking processing!\n");

	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	return 0;
}

int main()
{
	kbt();
	uint32_t device_count = k4a_device_get_installed_count();
	printf("Found %d connected devices.\n", device_count);
	k4a_device_t device = NULL;
	for(uint32_t device_index = 0; device_index < device_count; device_index++){
		if(k4a_device_open(device_index, &device) != K4A_RESULT_SUCCEEDED){
			printf("%d: Failed to open device.", device_index);
			continue;
		}

		char* serial_number = NULL;
		size_t serial_number_length = 0;
		if(k4a_device_get_serialnum(device, NULL, &serial_number_length) != K4A_BUFFER_RESULT_TOO_SMALL){
			printf("%d: Failed to get serial number length\n", device_index);
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		serial_number = (char*)malloc(serial_number_length);
		if(serial_number == NULL){
			printf("%d: Failed to malloc memory\n", device_index);
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		if(k4a_device_get_serialnum(device, serial_number, &serial_number_length) != K4A_BUFFER_RESULT_SUCCEEDED){
			printf("%d: Failed to get serial number", device_index);
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		printf("%d: Device serial number %s\n", device_index, serial_number);

		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
		config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

		if(k4a_device_start_cameras(device, &config)){
			printf("Failed to start device!\n");
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		k4a_capture_t capture = NULL;
		while(1){
			switch(k4a_device_get_capture(device, &capture, 1000)){
				case K4A_WAIT_RESULT_SUCCEEDED:
					break;
				case K4A_WAIT_RESULT_TIMEOUT:
					printf("Timed out waiting for a capture\n");
					continue;
					break;
				case K4A_WAIT_RESULT_FAILED:
					printf("Failed to read a capture\n");
					goto Exit;
			}

			// k4a_capture_get_color_image()
			// k4a_capture_get_depth_image()
			// k4a_capture_get_ir_image()

			k4a_image_t image = k4a_capture_get_depth_image(capture);
			show_image_depth(image);
			k4a_capture_release(capture);
		}

Exit:
		k4a_device_close(device);
	}
	system("pause");
    return 0;
}

