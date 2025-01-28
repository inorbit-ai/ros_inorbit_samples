install(
  FILES
    test/sample_data/config.yaml
  DESTINATION share/${PROJECT_NAME}/test/sample_data
)

install(
  DIRECTORY test/sample_data/hardcodedRosbag/
  DESTINATION share/${PROJECT_NAME}/test/sample_data/hardcodedRosbag
)
