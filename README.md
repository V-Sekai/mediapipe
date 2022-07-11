# Mediapipe OSC

See the github actions for build instructions.

Build the binary.

```
bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/holistic_tracking:holistic_tracking_cpu
```

Watch a webcam.

`bazel-bin/mediapipe/examples/desktop/holistic_tracking/holistic_tracking_cpu --calculator_graph_config_file=mediapipe/graphs/holistic_tracking/holistic_tracking_cpu.pbtxt`

Download a creative commons video.

`yt-dlp.exe https://www.youtube.com/watch?v=ypkrDdd61wk --recode-video mp4` 

> Aalaya Pilates
> Latihan Sakit Otot Pinggang  - Semua Pilates 30min - Meregangkan sekaligus melatih otot punggung
> Creative Commons Attribution

Play a video.

```
.\bazel-bin\mediapipe\examples\desktop\holistic_tracking\holistic_tracking_cpu.exe --calculator_graph_config_file=mediapipe\modules\holistic_landmark\holistic_world_landmark_cpu.pbtxt --input_video_path "..\Latihan Sakit Otot Pinggang  - Semua Pilates 30min - Meregangkan sekaligus melatih otot punggung [ypkrDdd61wk].mp4"
```

Was inspired by https://github.com/infosia/vmc2bvh.

## Blender OSC stream



## References

See also https://github.com/wongfei/ue4-mediapipe-plugin