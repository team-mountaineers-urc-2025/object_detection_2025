# object_detection_2025
Implements YOLO model for detection of hammer and water bottle

### Library Issues
If ultralytics is not installed it will not work on that device. Some issues may arrise in torch or numpy versions.

*Specifically the numpy version must be less than 2.0. I would sugguest version 1.26.4*

`pip install ultralytics`

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `camera_metadata` | String | '/CM_cam_meta' | Topic name for the camera's metadata |
| `cam_images` | String | '/image_topic' | Topic name for the camera's image |
| `cam_object` | String | '/cam_object_pose' | Topic name for the published object position |

### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `/image_topic` | Image | The images being sent by the Camera Manager |
| `/CM_cam_meta` | ImageMetadata | The image metadata being sent by the Camera Manager. Includes focal length, sensor height, image height, and image width. |

### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `/cam_object_pose` | ObjectPoint | Publishes the position of the object(hammer/bottle) relative to the camera that identified it. |
