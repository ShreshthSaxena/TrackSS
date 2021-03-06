//
//  test_frames.swift
//  SORT
//
//  Created by Shreshth Saxena on 24/09/20.
//

import Foundation
import KalmanFilter

/**
 Computes IUO between two bboxes in the form [l,t,w,h]
 */
public func iou(bb_test:Array<Double>,bb_gt:Array<Double>) -> Double{
    let xx1 = max(bb_test[0], bb_gt[0])
    let yy1 = max(bb_test[1], bb_gt[1])
    let xx2 = min(bb_test[2], bb_gt[2])
    let yy2 = min(bb_test[3], bb_gt[3])
    let w = max(0, xx2 - xx1)
    let h = max(0, yy2 - yy1)
    let wh = w * h
    let o = Double(wh) / Double(((bb_test[2]-bb_test[0])*(bb_test[3]-bb_test[1])
      + (bb_gt[2]-bb_gt[0])*(bb_gt[3]-bb_gt[1]) - wh))
    return(o)
}

/**
Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
 */
public func convert_bbox_to_z(bbox_int:Array<Double>)->Array<Double>{
  let bbox = bbox_int.map({ Double($0) })
  let w = bbox[2] - bbox[0]
  let h = bbox[3] - bbox[1]
  let x = bbox[0] + w/2
  let y = bbox[1] + h/2
  let s = w * h //scale = area
  let r = w / h
    return [x, y, s, r]
}

/**
 Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
     [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 */
public func convert_x_to_bbox(x:Array<Double>)->Array<Double>{
    let w = sqrt(Double(x[2] * x[3]))
    let h = x[2] / w
    return [x[0]-w/2,x[1]-h/2,x[0]+w/2,x[1]+h/2]
}

var lastId = 0

/**
 This class represents the internal state of individual tracked objects observed as bbox.
 */
public class KalmanTracker{
    public var id:Int
    public var hits:Int
    public var hit_streak:Int
    public var age:Int
    public var time_since_update:Int
    public var history:Array<Array<Double>>
    public var x:KMatrix   //state vector
    public var P:KMatrix  //initial state uncertainity
    public let B:KMatrix   //control matrix
    public let u:KMatrix   //control vector
    public let F:KMatrix   //next state fn
    public let H:KMatrix   //measurement fn
    public var R:KMatrix   //mesurement uncertainity
    public let Q:KMatrix  //process uncertainity
    public lazy var kalmanFilter = KalmanFilter(stateEstimatePrior: x, errorCovariancePrior: P)
    
    /**
     Initialises a tracker using initial bounding box.
     */
    public init(bbox: Array<Double>){
        self.x = KMatrix(grid: convert_bbox_to_z(bbox_int: bbox)+[0,0,0], rows: 7, columns: 1)
        self.P = KMatrix(grid: [10,0,0,0,0,0,0, 0,10,0,0,0,0,0, 0,0,10,0,0,0,0, 0,0,0,10,0,0,0, 0,0,0,0,10000,0,0, 0,0,0,0,0,10000,0, 0,0,0,0,0,0,10000], rows: 7, columns: 7)
        self.B = KMatrix(identityOfSize: 7)
        self.u = KMatrix(vector: [0, 0, 0, 0, 0, 0, 0])
        self.F = KMatrix(grid: [1,0,0,0,1,0,0, 0,1,0,0,0,1,0, 0,0,1,0,0,0,1, 0,0,0,1,0,0,0, 0,0,0,0,1,0,0, 0,0,0,0,0,1,0, 0,0,0,0,0,0,1], rows: 7, columns: 7)
        self.H = KMatrix(grid: [1,0,0,0,0,0,0, 0,1,0,0,0,0,0, 0,0,1,0,0,0,0, 0,0,0,1,0,0,0], rows: 4, columns: 7)
        self.R = KMatrix(grid: [1,0,0,0, 0,1,0,0, 0,0,10,0, 0,0,0,10], rows: 4, columns: 4)
        self.Q = KMatrix(grid: [1,0,0,0,0,0,0, 0,1,0,0,0,0,0, 0,0,1,0,0,0,0, 0,0,0,1,0,0,0, 0,0,0,0,0.01,0,0, 0,0,0,0,0,0.01,0, 0,0,0,0,0,0,0.0001], rows: 7, columns: 7)
        self.id = lastId; lastId+=1
        self.time_since_update = 0
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        self.history = []
        self.kalmanFilter = KalmanFilter(stateEstimatePrior: self.x, errorCovariancePrior: self.P)
    }
    
    public func describe(){
        print("id:",self.id, "bbox:",self.get_state(), "time_since_update:",self.time_since_update, "hits:",self.hits, "hit_streak:", self.hit_streak, "age:", self.age)
    }
    
    /**
     Updates the state vector with observed bbox.
     */
    public func update(bbox: Array<Double>){
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        let z = KMatrix(grid: convert_bbox_to_z(bbox_int: bbox), rows: 4, columns: 1)
        self.kalmanFilter = self.kalmanFilter.update(measurement: z, observationModel: H, covarienceOfObservationNoise: R)
    }
    
    /**
     Advances the state vector and returns the predicted bounding box estimate.
     */
    public func predict()->Array<Double>{
        self.kalmanFilter = self.kalmanFilter.predict(stateTransitionModel: self.F, controlInputModel: self.B, controlVector: self.u, covarianceOfProcessNoise: self.Q)
        self.age+=1
        if self.time_since_update > 0{ self.hit_streak = 0 }
        self.time_since_update+=1
        self.history.append(convert_x_to_bbox(x: self.kalmanFilter.stateEstimatePrior.grid))
        return self.history.last!
    }
    
    public func get_state()->Array<Double>{
        return convert_x_to_bbox(x: self.kalmanFilter.stateEstimatePrior.grid)
    }
    
}

/**
 Assigns detections to tracked object (both represented as bounding boxes)
 Returns 3 lists of matches, unmatched_detections and unmatched_trackers
 */
public func associate_detections_to_trackers(detections: Array<Array<Double>>,trackers: Array<Array<Double>>,iou_threshold:Double = 0.3) -> ([(Int,Int)],[Int],[Int]){
    if trackers.count == 0{
        return ([],Array(0..<detections.count),[])
    }
    
    var iou_matrix:[[Double]] = Array(repeating: Array(repeating: 0, count: trackers.count), count: detections.count)
    
    for (d,det) in detections.enumerated(){
        for (t,trk) in trackers.enumerated(){
            iou_matrix[d][t] = iou(bb_test: det,bb_gt: trk)
        }
    }
    let h = HunSolver(matrix: iou_matrix, maxim: true)
    guard let matched_indices = h?.solve() else{
        return ([],Array(0..<detections.count),Array(0..<trackers.count))
    }
    var unmatched_detections:[Int] = []
    var unmatched_trackers:[Int] = []
    
    //filter out matched with low IOU
    var matches:[(Int,Int)] = []
    for m in matched_indices.1{
        if m.1 >= trackers.count{
            unmatched_detections.append(m.0)
        }
        else if m.0 >= detections.count{
            unmatched_trackers.append(m.1)
        }
        else if iou_matrix[m.0][m.1] < iou_threshold{
            unmatched_detections.append(m.0)
            unmatched_trackers.append(m.1)
        }
        else{
            matches.append(m)
        }
    }
    return (matches, unmatched_detections, unmatched_trackers)
}

public class TrackerSS{
    public var trackers:Array<KalmanTracker>
    public var min_hits: Int
    public var max_age: Int
    public var frame_count:Int
    public var more_than_one_active = false
    public var patience = 0
    public var creationTime: Date
    public var iou_threshold:Double
    
    /**
     Sets key parameters for SORT
     */
    public init(max_age: Int=1 ,min_hits: Int=3, iou_threshold:Double = 0.3){
        self.trackers = [] //depends on where you invoke SORT
        self.min_hits = min_hits
        self.max_age = max_age
        self.frame_count = 0
        self.iou_threshold = iou_threshold
        self.creationTime = Date()
        lastId = 0
    }
    
    /**
       dets - an Int array of detections in the format [[x1,y1,x2,y2],[x1,y1,x2,y2],...]
       Requires: this method must be called once for each frame .
       Returns an array, where the last column is the object ID.
       NOTE: The number of objects returned may differ from the number of detections provided.
     */
    public func update(dets: Array<Array<Double>>)->Array<Array<Double>>{
        var ret:Array<Array<Double>> = []
        self.frame_count += 1
        
        let trks = self.trackers.map({$0.predict()})
        let (matched,unmatched_dets,_) = associate_detections_to_trackers(detections: dets, trackers: trks)
//        print("matches",matched)
        
        for m in matched{
            self.trackers[m.1].update(bbox: dets[m.0]) //update bbox
        }
        
        for i in unmatched_dets{ self.trackers.append(KalmanTracker(bbox: dets[i])) }
        
        var i = self.trackers.count
        for trk in self.trackers.reversed(){
            let d = trk.get_state()
            if trk.time_since_update < 1 && (trk.hit_streak >= self.min_hits || self.frame_count <= self.min_hits){
                ret.append(d+[Double(trk.id+1)])
            }
            i-=1
            if trk.time_since_update>self.max_age{
                self.trackers.remove(at: i)
            }
        }
//        print("trackers", self.trackers.count)
        
        if ret.count > 0{
            return ret
        }
        return []
        
    }
    
}



