# AVSA ball tracker

## Task 3.3: 

run as :

 ./main /home/sebasmos/AVSA2020datasets/AVSA_Lab3_datasets/dataset_lab3
 
 
## Comments

### INITIAL SETTING 

*abandonedBox_600_1000_clip.mp4*: 
From this image there are a ot of false positive at the beggining of the scene, due to illumination problems probably, because there was an initial prediction pointing at a random direction, when the background was totally static. 

A ciclist appears and immediatly the correction starts to work, as a reference we also have plotted in one of the windows the bounding boxes respectively, so it is possible to observe where the center will be determine. This is important to notice because the bbox determines the tracking direction.

When the ciclist stops riding, the predictor starts to predict and probably because of the high covariance, the system fails and predicts a random direction when instead the cyclist was at stop position. 

Remarks: The blobs detector works almost good for detecting people. Therefore a fine-tuned classification for objects will help to obtain better measurements, since they will be better centered. 

Then


*boats_6950_7900_clip.mp4*: 

Using initial configuration without tuning: From this video there are several detected elements, considering trees, the grass, the leaves, the waves on the water. From here, we understand that the foreground extraction is not very precise, there is a lot of noise and false positives.

One of the key elements is to understand what are we intending to track. On this video, we need to track the boat, changes on kernel, blobs size and  and tuning of the foreground mask. After, the blobs must be tuned (obj = boat) with correct sizes, since from this depends the measurement and therefore the prediction task and tracking at each frame step. 

Another problem: In this sequence we count with other objects on the same image, therefore it becomes into a more tedious homework for the tracker because we are assigning only one measurement, since we are expecting to track a single object, not multiple object. This is the reason why so suddenly, the drawing pointer starts to jump from one extreme position to another one, since new objects can be detected at different times.  

Fix: blob tuning for big objects such as in this case, to neglect waves and too small other objects (birds, people behind the main obj, etc)

*pedestrians_800_1025_clip.mp4*: 

Highly lighted image, mostly the ground. 

The blobs are detecting mainly the foot or is part-based between feet and other parts of the body. 
Thanks to the foreground segmentation we can observe where the blobs should be detected, in this case we have a shadow problem, therefore blobs are detected here and therefore the tracker fails to recognize the person until it the shadow disappears, moment at which the tracker identifies the person again, as well as the corrections.

*streetCornerAtNight_0_100_clip.mp4*:

We have some illumniation problems, with the lighning of a motorbike and the reflection of this one onto the ground. So this illumination confuses the system and then blobs are the detected over the reflections and ultimately, the tracker also predicts points on this reflections instead of identifying the object (a motorbike

### MODIFY OEM AND KF FIXED
*abandonedBox_600_1000_clip.mp4*: 

1. Fix logic when no blobs are detected along a period of time. Threshold of 10 blobs is set using boolean flags, therefore, when no blobs are detected in more than 10 frames, then the prediction will be equal to the current measurement, because due to EOM and GMM, moving objects become stationary by running average after some few seconds, and therefore a new condition must be set to differentiaty this stationary-transition state, to avoid confusions with occlusions. 
2. The smaller the kernel sizes, the worst the predictions, because there will be a lot of noise which will be erronously be detected as blobs, worsening the measurement and therefore the predictions and corrections, on which case, a high variance P and R would be necessary. Kernel sizes from 1 to 6 were tested, yielding kernel size 6 as the best, neglecting body-parts blobs and avoiding false positives. Also using kernel 5, there are more body-parts blobs detected on the cyclist, but the legs from a person on the top right are also detected. 


