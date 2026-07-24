def tum_publisher() -> None:
    from rgbd_odometry_ros.tum_rgbd_publisher import main

    main()


def odometry() -> None:
    from rgbd_odometry_ros.rgbd_odometry_py_node import main

    main()
