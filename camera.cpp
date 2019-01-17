//SM
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include "json11.hpp"
//QUEUE
#include <mqueue.h>
#include <string>
//OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;
using namespace aruco;
using namespace json11;

//SM
#define handle_error(msg) do { perror(msg); exit(EXIT_FAILURE); } while (0)
#define FILE_PERMS 0777
sem_t * sem_id_gps;
sem_t * sem_id_cmd;
long shm_size;

int open_shm_file(const char *fname){
    int shm_fd;
    shm_size = sysconf(_SC_PAGESIZE);

    const char *path = "/dev/shm/";
    char fpath[100];
    // Train wreck waiting to happpen. If you really want to break this
    // you will probably find a way.
    int len = sprintf(fpath, "%s%s", path, fname);
    if ( len < 0)
        handle_error(strerror(errno));
    
    int access_file = access(fpath, F_OK | R_OK | W_OK);
    if(access_file < 0){
        // File does not exist. So let's create it, and set initial size.
        if (errno == 2){
            shm_fd = shm_open(fname, O_CREAT | O_RDWR, FILE_PERMS);
            ftruncate(shm_fd, shm_size);
        }
        else { handle_error(strerror(errno)); }
    }
    else { shm_fd = shm_open(fname, O_CREAT | O_RDWR, FILE_PERMS); }

    return shm_fd;
}

//Open the default video camera
VideoCapture cap(0);
int dBrightness, dContrast, dSaturation, dGain, dExposure;

static void on_brightness(int, void*)
{
	cap.set(CAP_PROP_BRIGHTNESS,dBrightness / 100.0);
}

static void on_contrast(int, void*)
{
	cap.set(CAP_PROP_CONTRAST, dContrast / 100.0);
}

static void on_saturation(int, void*)
{
	cap.set(CAP_PROP_SATURATION, dSaturation / 100.0);
}

static Point2f calculate_middle(vector<Point2f> corners)
{
	Point2f middle;
	middle.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4;
	middle.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4;
	return middle;
}

int main(int argc, char* argv[])
{
	sem_id_gps=sem_open("/GPS_SEM", O_CREAT, S_IRUSR | S_IWUSR, 0);	
	sem_id_cmd=sem_open("/CMD_SEM", O_CREAT, S_IRUSR | S_IWUSR, 0);	
	int shm_fd_gps = open_shm_file("GPS_SM");
	int shm_fd_cmd = open_shm_file("CMD_SM");
	char *mmap_addr_gps = (char*) mmap(NULL, shm_size, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd_gps, 0);
	char *mmap_addr_cmd = (char*) mmap(NULL, shm_size, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd_cmd, 0);
	mqd_t mq = mq_open("/CAMERA_Q", O_WRONLY);

	// if not success, exit program
	if (cap.isOpened() == false)
	{
		cout << "Cannot open the video camera" << endl;
		cin.get(); //wait for any key press
		return -1;
	}
	
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	int dWidth = cap.get(CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	int dHeight = cap.get(CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	dBrightness = cap.get(CAP_PROP_BRIGHTNESS) * 100;
	dContrast = cap.get(CAP_PROP_CONTRAST) * 100;
	dSaturation = cap.get(CAP_PROP_SATURATION) * 100;
	dGain = cap.get(CAP_PROP_GAIN) * 100;
	dExposure = cap.get(CAP_PROP_EXPOSURE) * 100;

	cout << "Resolution of the video : " << dWidth << " x " << dHeight << endl;
	cout << "Brightness: " << dBrightness << endl;
	cout << "Contrast: " << dContrast << endl;
	cout << "Saturation: " << dSaturation << endl;
	cout << "Gain: " << dGain << endl;
	cout << "Exposure: " << dExposure << endl;

	string window_name = "My Camera Feed";
	string camera_control_name = "Camera control";
	string window_name2 = "Detected";

	namedWindow(camera_control_name, WINDOW_AUTOSIZE);
//	namedWindow(window_name2);

	createTrackbar("Brightness", "Camera control", &dBrightness, 100, on_brightness);
	createTrackbar("Contrast", "Camera control", &dContrast, 100, on_contrast);
	createTrackbar("Saturation", "Camera control", &dSaturation, 100, on_saturation);

	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	Ptr<DetectorParameters> parameters = DetectorParameters::create();
	//Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_1000);
	Dictionary gen_dictionary;
	gen_dictionary.markerSize = 3;
	gen_dictionary.maxCorrectionBits = 3;
	Mat markerBits(3, 3, CV_8UC1);
	markerBits.at<uchar>(0, 0) = 0; markerBits.at<uchar>(0, 1) = 1; markerBits.at<uchar>(0, 2) = 0;
	markerBits.at<uchar>(1, 0) = 1; markerBits.at<uchar>(1, 1) = 1; markerBits.at<uchar>(1, 2) = 1;
	markerBits.at<uchar>(2, 0) = 0; markerBits.at<uchar>(2, 1) = 1; markerBits.at<uchar>(2, 2) = 0;
	Mat markerCompressed = Dictionary::getByteListFromBits(markerBits);
	gen_dictionary.bytesList.push_back(markerCompressed);
	Ptr<Dictionary> dictionary = (Ptr<Dictionary>)&gen_dictionary; //Sypie przy wyłączaniu

	bool loop_stop = false;
	int tmp = 0;
	namedWindow(window_name); //create a window called "My Camera Feed"
	while (!loop_stop)
	{
		Mat frame;
		bool bSuccess = cap.read(frame); // read a new frame from video 
		
		//Breaking the while loop if the frames cannot be captured
		if (bSuccess == false)
		{
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			break;
		}

		//blur(frame, frame, Size(9, 9));
		detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		drawDetectedMarkers(frame, markerCorners, markerIds);

		if(markerCorners.size() > 0)
		{
			string err;
			sem_wait(sem_id_gps);
			const auto json_gps = Json::parse(mmap_addr_gps, err);
			sem_post(sem_id_gps);
			string pitch = json_gps["pitch"].dump();
			string time =  json_gps["time"].dump();
			string lat =  json_gps["lat"].dump();
			string alt = json_gps["alt"].dump();
			string  r_alt= json_gps["r_alt"].dump();
			string  lon = json_gps["lon"].dump();
			string  roll = json_gps["roll"].dump();
			string  yaw= json_gps["yaw"].dump();

			for (unsigned int i = 0; i < markerCorners.size(); i++)
			{
				Point2f middle = calculate_middle(markerCorners[i]);
				circle(frame, middle, 60, Scalar(255, 0, 0), 5, 8, 0);
				
				Json my_json = Json::object {
					{ "pitch", pitch },
					{ "time", time },
					{ "lat", lat },
					{ "alt", alt },
					{ "r_alt", r_alt },
					{ "lon", lon },
					{ "roll", roll },
					{ "yaw", yaw },
					{ "x", middle.x },
					{ "y", middle.y},
					};
				string json_obj_str = my_json.dump();
				cout << json_obj_str << endl;
				mq_send(mq, json_obj_str.c_str(), json_obj_str.length(), 0);
			}
		}
		drawDetectedMarkers(frame, rejectedCandidates, noArray(), Scalar(100, 0, 255));
		//show the frame in the created window
		imshow(window_name, frame);

		switch (waitKey(2))
		{
		case 113://Q
			tmp = cap.get(CAP_PROP_BRIGHTNESS);
			cap.set(CAP_PROP_BRIGHTNESS, ++tmp);
			dBrightness = cap.get(CAP_PROP_BRIGHTNESS);
			cout << "Brightness: " << dBrightness << endl;
			break;
		case 97://A
			tmp = cap.get(CAP_PROP_BRIGHTNESS);
			cap.set(CAP_PROP_BRIGHTNESS, --tmp);
			dBrightness = cap.get(CAP_PROP_BRIGHTNESS);
			cout << "Brightness: " << dBrightness << endl;
			break;
		case 119://W
			tmp = cap.get(CAP_PROP_CONTRAST);
			cap.set(CAP_PROP_CONTRAST, ++tmp);
			dContrast = cap.get(CAP_PROP_CONTRAST);
			cout << "Contrast: " << dContrast << endl;
			break;
		case 115://S
			tmp = cap.get(CAP_PROP_CONTRAST);
			cap.set(CAP_PROP_CONTRAST, --tmp);
			dContrast = cap.get(CAP_PROP_CONTRAST);
			cout << "Contrast: " << dContrast << endl;
			break;
		case 101://E
			tmp = cap.get(CAP_PROP_SATURATION);
			cap.set(CAP_PROP_SATURATION, ++tmp);
			dSaturation = cap.get(CAP_PROP_SATURATION);
			cout << "Saturation: " << dSaturation << endl;
			break;
		case 100://D
			tmp = cap.get(CAP_PROP_SATURATION);
			cap.set(CAP_PROP_SATURATION, --tmp);
			dSaturation = cap.get(CAP_PROP_SATURATION);
			cout << "Saturation: " << dSaturation << endl;
			break;
		case 114://R
			tmp = cap.get(CAP_PROP_GAIN);
			cap.set(CAP_PROP_GAIN, ++tmp);
			dGain = cap.get(CAP_PROP_GAIN);
			cout << "Gain: " << dGain << endl;
			break;
		case 102://F
			tmp = cap.get(CAP_PROP_GAIN);
			cap.set(CAP_PROP_GAIN, --tmp);
			dGain = cap.get(CAP_PROP_GAIN);
			cout << "Gain: " << dGain << endl;
			break;
		case 116://T
			tmp = cap.get(CAP_PROP_EXPOSURE);
			cap.set(CAP_PROP_EXPOSURE, ++tmp);
			dExposure = cap.get(CAP_PROP_EXPOSURE);
			cout << "Exposure: " << dExposure << endl;
			break;
		case 103://G
			tmp = cap.get(CAP_PROP_EXPOSURE);
			cap.set(CAP_PROP_EXPOSURE, --tmp);
			dExposure = cap.get(CAP_PROP_EXPOSURE);
			cout << "Exposure: " << dExposure << endl;
			break;
		case 27:
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			loop_stop = true;
			break;
		case -1:
			break;
		default:
			//cout << "Selected key: " << key << endl;
			break;
		}
	}

	cap.release();
	return 0;
}
