add_library(myslam SHARED 
        dataset.cpp
        config.cpp
        camera.cpp
        frame.cpp
        feature.cpp
        backend.cpp
        frontend.cpp
        visual_odometry.cpp
        map.cpp
        viewer.cpp
        mappoint.cpp
        )

target_link_libraries(myslam ${THIRD_PARTY_LIBS})