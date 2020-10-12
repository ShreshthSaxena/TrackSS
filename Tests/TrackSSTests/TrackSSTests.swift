import XCTest
@testable import TrackSS

final class TrackSSTests: XCTestCase {
    func testExample() {
        // This is an example of a functional test case.
        let T = TrackerSS()
        var i = 0
        for f in frames{
            print("/////new frame", f.count, i); i+=1
            let ops = T.update(dets: f)
            for op in ops{ print(op)}
        }
    }

    static var allTests = [
        ("testExample", testExample),
    ]
}
