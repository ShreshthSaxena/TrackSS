//
//  test_frames.swift
//  SORT
//
//  Created by Shreshth Saxena on 24/09/20.
//

import Foundation
import KalmanFilter

func iou(bb_test:Array<Int>,bb_gt:Array<Int>) -> Double{
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

func associate_detections_to_trackers(detections: Array<Array<Int>>,trackers: Array<Array<Int>>,iou_threshold:Double = 0.3) -> ([(Int,Int)],[Int],[Int]){
//    if trackers.count == 0{
//        print("early return")
//    }
//    var iou_matrix:[[Double]] = []
    var iou_matrix:[[Double]] = Array(repeating: Array(repeating: 0, count: trackers.count), count: detections.count)
    
    for (d,det) in detections.enumerated(){
        for (t,trk) in trackers.enumerated(){
            iou_matrix[d][t] = Double(iou(bb_test: det,bb_gt: trk))
        }
    }
//    matched_indices = linear_assignment(-iou_matrix)
    let h = HunSolver(matrix: iou_matrix, maxim: true)
    guard let matched_indices = h?.solve() else{
        return ([],[0],[])
    }
    var unmatched_detections:[Int] = []
    
//    for d in 0..<detections.count{
//        if !matched_indices.1.map({$0.1}).contains(d){
//            unmatched_detections.append(d)
//        }
//    }
//
    var unmatched_trackers:[Int] = []
//    for t in 0..<trackers.count{
//        if !matched_indices.1.map({$0.0}).contains(t){
//            unmatched_trackers.append(t)
//        }
//    }
    
    print("matched_indices",matched_indices)
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
    print(matches)
    return (matches, unmatched_detections, unmatched_trackers)
}

class TrackSS{
    var trackers:Array<ball>
    var min_hits: Int?
    var max_age: Int?
    var frame_count:Int?
    var more_than_one_active = false
    var patience = 0
    var creationTime: Date?
    var x = KMatrix(grid: [0,0,0,0], rows: 4, columns: 1)  //[x,y]
    var P = KMatrix(grid: [1000, 0,0,0, 0, 1000,0,0,0,0,1000,0,0,0,0,1000], rows: 4, columns: 4)
    let B = KMatrix(identityOfSize: 4)
    let u = KMatrix(vector: [0, 0, 0, 0])
    let F = KMatrix(grid: [1,0,0.1,0,0,1,0,0.1,0,0,1,0,0,0,0,1], rows: 4, columns: 4) //next state fn
    let H = KMatrix(grid: [1, 0, 0, 0,0,1,0,0], rows: 2, columns: 4)   //measurement fn
    var R = KMatrix(grid: [1,0,0,1], rows: 2, columns: 2)     //mesurement uncertainity
    let Q = KMatrix(grid: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1], rows: 4, columns: 4)
    
    lazy var kalmanFilter = KalmanFilter(stateEstimatePrior: x, errorCovariancePrior: P)
    
    init(max_age: Int=3 ,min_hits: Int=3){
        self.trackers = [] //depends on where you invoke SORT
//          self.min_hits = min_hits
//          self.max_age = max_age
        self.frame_count = 0
        self.creationTime = Date()
    }
    
    func update(dets: Array<Array<Int>>){
        self.frame_count! += 1
        print("dets ",dets)
        if self.frame_count == 1 || (trackers.count == 0 && dets.count == 0){
            for det in dets{
                self.trackers.append(ball(bbox: det))
            }
            print("trackers ",trackers)
            return
        }
        
        let trks = trackers.map({$0.bbox})
        print("trackers no",trackers.count)
        print("det no",dets.count)
        let (matched,unmatched_dets,unmatched_trks) = associate_detections_to_trackers(detections: dets, trackers: trks)
        
//        for m in matched{
////            if !unmatched_trks.contains(t){
//            trackers[m.1].update(bbox: dets[m.0]) //update bbox
////            trackers[m.1].describe()
////            }
//        }
        for m in 0..<trackers.count{
            if let i = matched.first(where: {$0.1 == m}){
                trackers[m].update(bbox: dets[i.0])
            }
            else{
                trackers[m].time_since_update += 1
            }
            trackers[m].describe()
        }
        
        
        switch unmatched_dets.count{
        case 0:
            patience = 0
            if trackers.last?.state == .active{   //contains(where: { $0.state == .active }){
                // active tracker at last always
                if trackers.last!.time_since_update > 2{
                    trackers.last?.state = .lost
                }
                else if trackers.last?.time_since_update == 0{
                    trackers.last?.state = .tracked
                }
            }
            
            //delete Kalman Object
            //check for stale if matches.count > 0
        case 1:
            patience = 0
            if !trackers.contains(where: { $0.state == .active }){ //unmatched_trks.count == 0
                print("start kalman")
                let new_ball = ball(bbox: dets[unmatched_dets], state: .active)
                self.trackers.append(new_ball)
                kalmanFilter = KalmanFilter(stateEstimatePrior: x, errorCovariancePrior: P)
                let z = KMatrix(grid: [Double(new_ball.center.0), Double(new_ball.center.1)], rows: 2, columns: 1)
                kalmanFilter = kalmanFilter.update(measurement: z, observationModel: H, covarienceOfObservationNoise: R)
                kalmanFilter = kalmanFilter.predict(stateTransitionModel: F, controlInputModel: B, controlVector: u, covarianceOfProcessNoise: Q)
            }
            else{ //if trackers.last!.state == .active{
                //predict kalman
                //update kalman
                let center = ball(bbox: dets[unmatched_dets[0]]).center
                let z = KMatrix(grid: [Double(center.0), Double(center.1)], rows: 2, columns: 1)
                kalmanFilter = kalmanFilter.update(measurement: z, observationModel: H, covarienceOfObservationNoise: R)
                kalmanFilter = kalmanFilter.predict(stateTransitionModel: F, controlInputModel: B, controlVector: u, covarianceOfProcessNoise: Q)
//                let midx = Int(kalmanFilter.stateEstimatePrior.grid[0])
//                let midy = Int(kalmanFilter.stateEstimatePrior.grid[1])
                print("Kpred",kalmanFilter.stateEstimatePrior.grid)
                trackers.last!.update(bbox:dets[unmatched_dets[0]]) //[midx-10,midy-10,midx+10,midy+10]) //might need correction
            }
//            else{
//                print("more than one unmatched tracker")
//            }
        default:
            patience += 1
            print("ending session in \(5-patience)")
            if patience>5{
                more_than_one_active = true //end or pause session
            }
        }
        
    }
    
}


enum State {
    case active,inactive,lost,tracked
}
var lastId = 1

class ball{
    var state:State?
    var id:Int
    var bbox:Array<Int>
    var center:(Int,Int)
    var hits:Int
    var time_since_update:Int
    
    init(bbox: Array<Int>,state:State = .inactive){
        self.state = state
        self.id = lastId; lastId+=1
        self.bbox = bbox
        self.center = (bbox[0]+(bbox[2]-bbox[0])/2 , bbox[1]+(bbox[3]-bbox[1])/2)
        self.time_since_update = 0
        self.hits = 0
    }
    
    func describe(){
        print("id:",self.id,"box:", self.bbox, "center:", self.center, "state:", self.state as Any, "time_since_update:",self.time_since_update, "hits:",self.hits)
    }
    
//    func transition(){
//        print(self.state!)
//    }

    func update(bbox: Array<Int>){
        self.time_since_update = 0
        self.hits += 1
        self.bbox = bbox
        self.center = (bbox[0]+(bbox[2]-bbox[0])/2 , bbox[1]+(bbox[3]-bbox[1])/2)
    }
}
