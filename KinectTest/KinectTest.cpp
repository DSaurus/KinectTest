// KinectTest.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "k4a/k4a.h"
#include <iostream>



int main()
{
	uint32_t device_count = k4a_device_get_installed_count();
	printf("Found %d connected devices.\n", count);
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

		serial_number = malloc(serial_number_length);
		if(serial_number == NULL){
			printf("%d: Failed to malloc memory\n", device_index);
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		if(k4a_device_get_serialnum(device, serial_number, serial_number_length) != K4A_BUFFER_RESULT_SUCCEEDED){
			printf("%d: Failed to get serial number", device_index);
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		printf("%d: Device serial number %s\n", serial_number);

		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.color_format = K4A_IAMGE_FORMAT_COLOR_MJPG;
		config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

		if(k4a_device_start_camera(device, &config)){
			printf("Failed to start device!\n");
			k4a_device_close(device);
			device = NULL;
			continue;
		}

		k4a_capture_t capture = NULL;
		while(1){
			switch(k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS)){
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
			if(image != NULL){
				printf("| Depth16 res:%4dx%4d stride:%5d\n", 
					k4a_image_get_height_pixels(image),
					k4a_image_get_width_pixels(image),
					k4a_image_get_stride_bytes(image));
			}
			
			k4a_capture_release(capture);
		}

Exit:
		k4a_device_close(device);
	}
	system("pause");
    return 0;
}

