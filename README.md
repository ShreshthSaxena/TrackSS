# TrackSS

[SORT](http://arxiv.org/abs/1602.00763) based Object Tracker in Swift <br>
for author's Python implementation check : https://github.com/abewley/sort

## Introduction

SORT is a barebones implementation of a visual multiple object tracking framework based on rudimentary data association and state estimation techniques. It is designed for online tracking applications where only past and current frames are available and the method produces object identities on the fly. While this minimalistic tracker doesn't handle occlusion or re-entering objects its purpose is to serve as a baseline and testbed for the development of future trackers.

SORT was initially described in [this paper](http://arxiv.org/abs/1602.00763). At the time of the initial publication, SORT was ranked the best *open source* multiple object tracker on the [MOT benchmark](https://motchallenge.net/results/2D_MOT_2015/).

Dependancy Credits: 
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

- make corrections/improvements as needed
- add support for Carthage and Cocoapods
- Benchmark on MOT datasets
