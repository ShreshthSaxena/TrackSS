import XCTest
@testable import TrackSS

final class TrackSSTests: XCTestCase {
    
//    func testExample() {
//        let T = TrackerSS()
//        var i = 0
//        for f in frames{
//            print("/////new frame", f.count, i); i+=1
//            let ops = T.update(dets: f)
//            for op in ops{ print(op)}
//        }
//    }
    
    func testMOT() {
        var total_time:Double = 0
        var total_frames = 0
        
        do{
            let fileurls = try FileManager.default.contentsOfDirectory(at: URL(string: "/Users/xx/Documents/data")!, includingPropertiesForKeys: nil)
//            let fname = fileurls.first
            for fname in fileurls.filter({ $0.pathExtension == "txt"}){
                let T = TrackerSS()
                let textcontent = try String(contentsOf: fname)
                print("processing",fname.lastPathComponent)
                var arr = textcontent.components(separatedBy: "\n").map({ $0.components(separatedBy: ",")})
                arr.removeLast()
                let no_of_frames = arr.map({ Int($0[0])! }).max()
                for frame in 1...no_of_frames!{
//                    let str = arr.filter({ Int($0[0]) == frame })
//                    let int = str.map({$0[2..<7]})
                    total_frames += 1
                    var dets:Array<Array<Double>> = []
                    for row in arr.filter({ Int($0[0]) == frame }){
                        let temp = row[2..<7].map({ Double($0)! })
                        dets.append([temp[0],temp[1],temp[2]+temp[0],temp[3]+temp[1]])
                    }
                    let start_time = DispatchTime.now()
                    let ops = T.update(dets: dets)
                    let cycle_time = Double(DispatchTime.now().uptimeNanoseconds - start_time.uptimeNanoseconds)/1000000000
                    total_time += cycle_time
//                    for op in ops{ print(op) } // or to output.txt
                }
            }
            print("Total Tracking took: \(total_time) seconds for \(total_frames) frames or \(Double(total_frames)/total_time) FPS")
        } catch let error{ print("ERROR", error) }
    }
    

    static var allTests = [
        ("testMOT", testMOT)
    ]
}
