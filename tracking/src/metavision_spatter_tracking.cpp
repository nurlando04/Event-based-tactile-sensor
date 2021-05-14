/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A. - All Rights Reserved                                                                 *
 *                                                                                                                    *
 * Subject to Prophesee Metavision Licensing Terms and Conditions ("License T&C's").                                  *
 * You may not use this file except in compliance with these License T&C's.                                           *
 * A copy of these License T&C's is located at docs.prophesee.ai/licensing and in the "LICENSE" file accompanying     *
 * this file.                                                                                                         *
 **********************************************************************************************************************/

// Example of using SDK to track simple, non colliding objects.

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <mutex>
#include <condition_variable>
#include <boost/program_options.hpp>
#include <metavision/sdk/analytics/utils/spatter_tracker_csv_logger.h>
#include <metavision/sdk/analytics/utils/events_frame_generator_algorithm.h>
#include <metavision/sdk/analytics/utils/tracking_drawing.h>
#include <metavision/sdk/analytics/events/event_spatter_cluster.h>
#include <metavision/sdk/analytics/configs/spatter_tracker_algorithm_config.h>
#include <metavision/sdk/analytics/algorithms/spatter_tracker_algorithm.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/events/event2d.h>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/utils/simple_displayer.h>
#include <metavision/sdk/driver/camera.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "simple_video_writer.h"
#include "simple_timer.h"

std::ofstream myfile;
std::ofstream myfile2;

// Utility namespace used to parse command line arguments
namespace boost_po = boost::program_options;
//ros::NodeHandle n;
/// @brief Class for spatter tracking pipeline
class Pipeline {
public:
    Pipeline() = default;

    ~Pipeline() = default;

    /// @brief Parses command line arguments
    bool parse_command_line(int argc, char *argv[]);

    float forces[3]={};

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("chatter", 1000);
    
    std_msgs::Float32 msg;

    std::stringstream ss;

    /// @brief Initializes the camera
    bool initialize_camera();

    /// @brief Initializes the filters
    void initialize_tracker();
    
    void roi(int left,int up, int right, int down);

    /// @brief Starts the pipeline and the camera
    bool start();

    /// @brief Stops the pipeline and the camera
    void stop();

    /// @brief Waits until the end of the file or until the exit of the display
    void run();

private:
    /// @brief Processes the output of the spatter tracker
    ///
    /// @param ts Current timestamp
    /// @param clusters Clusters to process
    void tracker_callback(Metavision::timestamp ts, const std::vector<Metavision::EventSpatterCluster> &clusters);

    // Spatter tracking parameters
    int cell_width_;           ///< Cell width used for clustering
    int cell_height_;          ///< Cell height used for clustering
    int activation_threshold_; ///< Number of events in a cell to consider it as active
    bool apply_filter_;        ///< If true, then the activation threshold considers only one event per pixel
    Metavision::timestamp accumulation_time_; ///< Processing accumulation time, in us
    int max_distance_;                        ///< Maximum distance for clusters association
    int untracked_threshold_; ///< Maximum number of times a cluster can stay untracked before being removed

    // Min object size to track
    int min_size_;
    // Max object size to track
    int max_size_;

    // Camera parameters
    std::string filename_    = "";
    std::string biases_file_ = "";

    // CSV filename to save the tracking output
    std::string res_csv_file_ = "";

    // Display parameters
    // If we display data on the screen or not
    bool display_ = true;
    // If display as fast as possible
    bool as_fast_as_possible_ = false;
    // Filename to save the resulted video
    std::string res_video_ = "";
    // If we want to measure computation time
    bool time_ = false;
    // Current image
    cv::Mat back_img_;

    // Conditional variables to notify the end of the processing
    std::condition_variable process_cond_;
    std::mutex process_mutex_;
    bool is_processing_ = true;

    Metavision::timestamp write_from_ = 0;
    Metavision::timestamp write_to_   = std::numeric_limits<Metavision::timestamp>::max();

    std::unique_ptr<Metavision::Camera> camera_;                                 ///< Pointer to Camera class
    std::unique_ptr<Metavision::SpatterTrackerAlgorithm> tracker_;               ///< Instance of the cluster maker
    std::unique_ptr<Metavision::SpatterTrackerCsvLogger> tracker_logger_;        ///< SpatterTracker csv logger
    std::unique_ptr<Metavision::EventsFrameGeneratorAlgorithm> frame_generator_; ///< Frame generator
    std::unique_ptr<SimpleVideoWriter> video_writer_;                            ///< Video writer
    std::unique_ptr<Metavision::SimpleDisplayer> displayer_;                     ///< Frame displayer
    std::unique_ptr<SimpleTimer> timer_;                                         ///< SpatterTracker timer
};

bool Pipeline::initialize_camera() {
    // If the filename is set, then read from the file
    if (filename_ != "") {
        try {
            camera_ =
                std::make_unique<Metavision::Camera>(Metavision::Camera::from_file(filename_, !as_fast_as_possible_));
        } catch (Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            return false;
        }
        // Otherwise, set the input source to the fist available camera
    } else {
        try {
            camera_ = std::make_unique<Metavision::Camera>(Metavision::Camera::from_first_available());

            if (biases_file_ != "") {
                camera_->biases().set_from_file(biases_file_);
            }

        } catch (Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            return false;
        }
    }

    // Add camera runtime error callback
    camera_->add_runtime_error_callback([](const Metavision::CameraException &e) { MV_LOG_ERROR() << e.what(); });

    return true;
}

bool Pipeline::parse_command_line(int argc, char *argv[]) {
    const std::string program_desc("Code sample using Metavision SDK to track simple, non colliding objects.\n"
                                   "By default, only ON events are tracked.\n");

    // clang-format off
    boost_po::options_description options_desc("Options");
    options_desc.add_options()
        ("help,h", "Produce help message.")
        ("input-raw-file,i",             boost_po::value<std::string>(&filename_), "Path to input RAW file. If not specified, the camera live stream is used.")
        ("biases,b",                     boost_po::value<std::string>(&biases_file_), "Path to a biases file. If not specified, the camera will be configured with the default biases")
        ("cell-width",                   boost_po::value<int>(&cell_width_)->default_value(7), "Cell width used for clustering, in pixels")
        ("cell-height",                  boost_po::value<int>(&cell_height_)->default_value(7), "Cell height used for clustering, in pixels")
        ("max-distance,D",               boost_po::value<int>(&max_distance_)->default_value(50), "Maximum distance for clusters association, in pixels")
        ("activation-threshold,a",       boost_po::value<int>(&activation_threshold_)->default_value(10), "Minimum number of events in a cell to consider it as active")
        ("apply-filter",                 boost_po::value<bool>(&apply_filter_)->default_value(true), "If true, then the cell activation threshold considers only one event per pixel")
        ("min-size",                     boost_po::value<int>(&min_size_)->default_value(20), "Minimum object size, in pixels")
        ("max-size",                     boost_po::value<int>(&max_size_)->default_value(50), "Maximum object size, in pixels")
        ("untracked-threshold",          boost_po::value<int>(&untracked_threshold_)->default_value(5),"Maximum number of times a cluster can stay untracked before being removed")
        ("processing-accumulation-time", boost_po::value<Metavision::timestamp>(&accumulation_time_)->default_value(1000),"Processing accumulation time (in us)")
        ("display,d",                    boost_po::value(&display_), "Activate display or not")
        ("out-video,o",                  boost_po::value<std::string>(&res_video_), "Path and name for saving the output slow motion .avi video")
        ("write-from,s",                 boost_po::value<Metavision::timestamp>(&write_from_), "Start time to save the video (in us)")
        ("write-to,e",                   boost_po::value<Metavision::timestamp>(&write_to_), "End time to save the video (in us)")
        ("time,t",                       boost_po::value(&time_), "Measure the time of processing in the interest time range")
        ("log-results,l",                boost_po::value<std::string>(&res_csv_file_), "File to save the output of tracking")
        ;

    // clang-format on

    boost_po::variables_map vm;
    try {
        boost_po::store(boost_po::command_line_parser(argc, argv).options(options_desc).run(), vm);
        boost_po::notify(vm);
    } catch (boost_po::error &e) {
        MV_LOG_ERROR() << program_desc;
        MV_LOG_ERROR() << options_desc;
        MV_LOG_ERROR() << "Parsing error:" << e.what();
        return false;
    }

    if (vm.count("help")) {
        MV_LOG_INFO() << program_desc;
        MV_LOG_INFO() << options_desc;
        return false;
    }

    MV_LOG_INFO() << options_desc;

    // If the display is enabled or if the data are taken from a live camera, then do not activate reproducing camera
    // behavior. Otherwise, if the display is disabled or if the data are taken from a RAW file, then process the
    // data as fast as possible
    if (!display_ || (display_ && !filename_.empty()))
        as_fast_as_possible_ = true;

    return true;
}


bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

bool Pipeline::start() {
    const bool started = camera_->start();
    
    if (!started)
        MV_LOG_ERROR() << "The camera could not be started.";


    int i=1;
    while(1){
        auto s = std::to_string(i);
        std::string ss="example"+s+".csv";
        std::string sss="distances"+s+".csv";
        const char * c=ss.c_str();
        const char * cc=sss.c_str();
        if (is_file_exist(c)){
            i++;
            continue;
        } 
        else{
            myfile.open(c);
            myfile2.open(cc);
            break;
        }
    }
    return started;
    
}

///void Pipleine::roi(int left,int up, int right, int down) {
///    Metavision::Roi
///}

void Pipeline::stop() {
    // Show the number of counted trackers
    MV_LOG_INFO() << "Counter =" << tracker_->get_cluster_count();

    try {
        myfile.close();
        myfile2.close();
        camera_->stop();
    } catch (Metavision::CameraException &e) { MV_LOG_ERROR() << e.what(); }
}

void Pipeline::run() {
    if (displayer_) {
        // Run the display in the main thread
        displayer_->run();
    } else {
        // Wait until the end of the file
        std::unique_lock<std::mutex> lock(process_mutex_);
        process_cond_.wait(lock, [this] { return !is_processing_; });
    }
}

float distance(int x1, int y1, int x2, int y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) +
                pow(y2 - y1, 2) * 1.0);
}
 
/// [TRACKING_TRACKER_CALLBACK_BEGIN]
void Pipeline::tracker_callback(Metavision::timestamp ts,
                                const std::vector<Metavision::EventSpatterCluster> &trackers) {
    if (tracker_logger_)
        tracker_logger_->log_output(ts, trackers);

    if (frame_generator_) {
        frame_generator_->generate_image_from_events(ts, back_img_);
        float alldist=0;
        for(int i=0;i<trackers.size();i++){
            for(int j=i+1;j<trackers.size();j++){
                alldist=alldist+distance(trackers[i].get_centroid().x,trackers[i].get_centroid().y,trackers[j].get_centroid().x,trackers[j].get_centroid().y);


            }
        }
        float trackF = 0;
        if (trackers.size()==1){
            trackF = 1.1;
        }else{
            trackF = trackers.size();
        }
        alldist=alldist/(trackF-1);
        float force =0;
        

        force=(alldist-201.5)/66;
        force=(forces[0]+forces[1]+forces[2]+force)/4;
        forces[0]=forces[1];
        forces[1]=forces[2];
        forces[2]=force;
        if(force<0.05){
            force=0;
        }
        if(force>1){
            force=1;
        }

        
        myfile2 << force << ",";
        myfile2 << ts << std::endl;


            for(int i=0; i<trackers.size(); i++){
 
                msg.data={force};
                chatter_pub.publish(msg);
                myfile << trackers[i].get_centroid().x << ",";
                myfile << trackers[i].get_centroid().y << ",";
                myfile << ts << std::endl;
        }
        
        Metavision::draw_tracking_results(ts, trackers.cbegin(), trackers.cend(), back_img_);
    }

    if (video_writer_)
        video_writer_->write_frame(ts, back_img_);

    if (displayer_)
        displayer_->swap_frame(back_img_);

    if (timer_)
        timer_->update_timer(ts);
}
/// [TRACKING_TRACKER_CALLBACK_END]

void Pipeline::initialize_tracker() {
    camera_->roi().set({150, 100, 500, 300});
    const auto &geometry    = camera_->geometry();
    ///geometry.roi().set({0, 0, 200, 200});
    const int sensor_width  = geometry.width();
    const int sensor_height = geometry.height();

    // Creates filterss
    Metavision::SpatterTrackerAlgorithmConfig tracker_config(cell_width_, cell_height_, accumulation_time_,
                                                             untracked_threshold_, activation_threshold_, apply_filter_,
                                                             max_distance_, min_size_, max_size_);

    tracker_ = std::make_unique<Metavision::SpatterTrackerAlgorithm>(sensor_width, sensor_height, tracker_config);

    tracker_->set_write_range(write_from_, write_to_);

    if (!res_csv_file_.empty()) {
        tracker_logger_.reset(new Metavision::SpatterTrackerCsvLogger(res_csv_file_));
    }

    if (!res_video_.empty() || display_) {
        frame_generator_.reset(new Metavision::EventsFrameGeneratorAlgorithm(sensor_width, sensor_height));
    }

    if (!res_video_.empty()) {
        video_writer_.reset(new SimpleVideoWriter(sensor_width, sensor_height, accumulation_time_, 30, res_video_));
        video_writer_->set_write_range(write_from_, write_to_);
    }

    if (display_) {
        displayer_.reset(new Metavision::SimpleDisplayer("Tracking result"));
        // Notify the pipeline when the displayer is exited
        displayer_->set_on_key_pressed_cb([this](int key) {
            if (key == 'q') {
                {
                    std::lock_guard<std::mutex> lock(process_mutex_);
                    is_processing_ = false;
                    process_cond_.notify_all();
                }
                displayer_->stop();
            }
        });
    }

    if (time_) {
        timer_.reset(new SimpleTimer());
        timer_->set_time_range(write_from_, write_to_);
    }

    /// [TRACKING_SET_CAMERA_CALLBACK_BEGIN]
    // Connects filters
    camera_->cd().add_callback([this](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        // Frame generator must be called first
        if (frame_generator_)
            frame_generator_->process_events(begin, end);

        tracker_->process_events(begin, end);
    });
    /// [TRACKING_SET_CAMERA_CALLBACK_END]

    /// [TRACKING_SET_OUTPUT_CALLBACK BEGIN]
    // Sets the callback to process the output of the spatter tracker
    tracker_->set_callback(
        [this](const Metavision::timestamp ts, const std::vector<Metavision::EventSpatterCluster> &clusters) {
            tracker_callback(ts, clusters);
        });
    /// [TRACKING_SET_OUTPUT_CALLBACK_END]

    // Stops the pipeline when the camera is stopped
    camera_->add_status_change_callback([this](const Metavision::CameraStatus &status) {
        if (status == Metavision::CameraStatus::STOPPED) {
            std::lock_guard<std::mutex> lock(process_mutex_);
            is_processing_ = false;

            if (displayer_)
                displayer_->stop();

            process_cond_.notify_all();
        }
    });
}

/// Main function
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "talker");
    Pipeline pipeline;

    
    // Parse command line
    if (!pipeline.parse_command_line(argc, argv))
        return 1;

    // Initialize the camera
    if (!pipeline.initialize_camera())
        return 1;

    // Initialize tracker
    pipeline.initialize_tracker();

    // Start the camera
    if (!pipeline.start())
        return 0;

    // Wait until the end of the pipeline
    pipeline.run();
    
    pipeline.stop();

    return 0;
}
