SET (PYTHON_FILE
    trajectory_publisher.py
    planning_request_adapter.py
    hpp_server_initializer.py
    client.py
    ros_tools.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus_hpp" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    hpp_server_initializer.py
    planning_request_adapter.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus_hpp/manipulation" ${F})
ENDFOREACH()
