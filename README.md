# anx_interface
```
anx_mock (python) <--anx_api--> anx_interface (python) --> anx_interface_ros2
```

## anx_mock
**Purpose:** Development and testing of anx_interface, anx_interface_ros and anx_interface_ros2

## Installing
**Note:** Install scons and poetry befor proceedings

```bash
cd anx_interface
scons
poetry build
pip install dist/anx_interface-<version>
```
