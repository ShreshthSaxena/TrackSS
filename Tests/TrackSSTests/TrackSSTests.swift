import XCTest
@testable import TrackSS

final class TrackSSTests: XCTestCase {
    func testExample() {
        // This is an example of a functional test case.
        // Use XCTAssert and related functions to verify your tests produce the correct
        // results.
//        XCTAssertEqual(TrackSS().text, "Hello, World!")
        let T = TrackSS()
        for f in frames{
            print("/////new frame")
            T.update(dets: f)
        }
    }

    static var allTests = [
        ("testExample", testExample),
    ]
}
