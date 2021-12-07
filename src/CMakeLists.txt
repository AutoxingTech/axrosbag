google_binary(axrosbag
  SRCS
    main_record.cpp
)

install(TARGETS axrosbag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)