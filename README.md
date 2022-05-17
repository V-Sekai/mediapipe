# Mediapipe OSC

See the github actions for build instructions.

Build the binary.

`bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/holistic_tracking:holistic_tracking_cpu`

Watch a webcam.

`GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/holistic_tracking/holistic_tracking_cpu --calculator_graph_config_file=mediapipe/graphs/holistic_tracking/holistic_tracking_cpu.pbtxt`

Download a creative commons video. # 

`yt-dlp.exe https://www.youtube.com/watch?v=J5aHzAJJBAA --recode-video mp4` 

> Tatiana Shilovskaya
> VOGUE TUTORIAL | #2 HANDS PERFORMANCE 1 PART | TATIANA HURRICANE
> Creative Commons Attribution

Play a video.

`GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/holistic_tracking/holistic_tracking_cpu --calculator_graph_config_file=mediapipe/graphs/holistic_tracking/holistic_tracking_cpu.pbtxt --input_video_path "VOGUE TUTORIAL _ #2 HANDS PERFORMANCE 1 PART _ TATIANA HURRICANE [J5aHzAJJBAA]"`