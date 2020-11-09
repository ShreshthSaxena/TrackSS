# TrackSS

[SORT](http://arxiv.org/abs/1602.00763) based Tracker for real-time Object Tracking in Swift <br>
for author's Python implementation check : https://github.com/abewley/sort


## Introduction

SORT is a barebones implementation of a visual multiple object tracking framework based on rudimentary data association and state estimation techniques. It is designed for online tracking applications where only past and current frames are available and the method produces object identities on the fly. While this minimalistic tracker doesn't handle occlusion or re-entering objects its purpose is to serve as a baseline and testbed for the development of future trackers.

SORT was initially described in [this paper](http://arxiv.org/abs/1602.00763). At the time of the initial publication, SORT was ranked the best *open source* multiple object tracker on the [MOT benchmark](https://motchallenge.net/results/2D_MOT_2015/).

Dependency Credits: 
- [Kalman Filter](https://github.com/wearereasonablepeople/KalmanFilter)
- [Hungarian method Solver](https://github.com/Jasagredo/Hume)


## Installation 

To add package in your Xcode project do: 
- File -> Swift Packages -> Add Package Dependency
- add url: **https://github.com/ShreshthSaxena/TrackSS/** and choose version/tag **1.0.0**

## Usage

- Create a Tracker object:
> let T = TrackerSS()

- Update tracker with detections on each timestep/frame:
>for f in frames{<br>
>    let res = T.update(dets: f)<br>
>    print(res)<br>
>}

- Input parameter **dets: Array<Array\<Int>>** is an array of detections in the format [[x1,y1,x2,y2],[x1,y1,x2,y2],...]

- Output returned is a similar 2d-array appended with object ID in last column.

## To do

- [x] Benchmark on provided datasets
- [ ] Add utility functions for Vision structs (CGRect/CGPoint)
- [ ] Add support for Carthage and Cocoapods

## Test outputs

Results of author's Python version and this Swift implementation on provided dataset ([data](data) folder) on Core-i5 MBP-2019.

Python output:<br>
> Processing ETH-Bahnhof.<br>
> Processing ADL-Rundle-8.<br>
> Processing ADL-Rundle-6.<br>
> Processing ETH-Pedcross2.<br>
> Processing TUD-Stadtmitte.<br>
> Processing TUD-Campus.<br>
> Processing KITTI-17.<br>
> Processing PETS09-S2L1.<br>
> Processing Venice-2.<br>
> Processing ETH-Sunnyday.<br>
> Processing KITTI-13.<br>
> Total Tracking took: 6.033 seconds for 5500 frames or 911.6 FPS<br>

Swift(Xcode) output:<br>
> processing ADL-Rundle-6.txt<br>
> processing TUD-Campus.txt<br>
> processing ETH-Sunnyday.txt<br>
> processing KITTI-17.txt<br>
> processing ETH-Pedcross2.txt<br>
> processing KITTI-13.txt<br>
> processing TUD-Stadtmitte.txt<br>
> processing Venice-2.txt<br>
> processing PETS09-S2L1.txt<br>
> processing ETH-Bahnhof.txt<br>
> processing ADL-Rundle-8.txt<br>
> Total Tracking took: 8.434 seconds for 5500 frames or 652.14 FPS<br>
