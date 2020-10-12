Pod::Spec.new do |spec|
spec.name         = "TrackSS"
spec.version      = "1.0.0"
spec.summary      = "Sort based Object Tracker in Swift"
spec.description  = <<-DESC
open to any improvements/corrections
DESC
spec.license      = { :type => "MIT", :file => "LICENSE" }
spec.author             = { "author" => "shreshth.saxena@outlook.com" }
spec.platforms = { :ios => "13.0", :osx => "10.15", :watchos => "6.0" }
spec.swift_version = "5.3"
spec.source       = { :git => "https://github.com/ShreshthSaxena/TrackSS.git", :tag => "#{1.0.0}" }
spec.source_files  = "Sources/TrackSS/**/*.swift"
spec.xcconfig = { "SWIFT_VERSION" => "5.3" }
spec.dependency '{ :git => "https://github.com/ShreshthSaxena/KalmanFilter.git", :tag => "#{1.0.0}" }'
spec.dependency '{ :git => "https://github.com/ShreshthSaxena/Upsurge.git", :tag => "#{1.0.0}" }'
end
