"""Parse installed launch/URDF configuration without starting any ROS process."""
import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchContext

ROOT=Path(__file__).resolve().parents[1]


def test_all_python_and_yaml_parse():
    for path in (ROOT/'src').rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    for base in ('src','config'):
        for path in (ROOT/base).rglob('*.yaml'):
            yaml.safe_load(path.read_text())


def test_package_dependencies_are_unique():
    for path in (ROOT/'src').glob('*/package.xml'):
        tree=ET.parse(path).getroot()
        dependencies=[(child.tag,child.text) for child in tree if 'depend' in child.tag]
        assert len(dependencies)==len(set(dependencies)), path
        assert tree.findtext('license') and 'TODO' not in tree.findtext('license')


@pytest.mark.parametrize('color',['blue','silver'])
def test_urdf_tree_and_resources(color):
    share=Path(get_package_share_directory('oarbot_description'))
    xml=xacro.process_file(str(share/'urdf'/f'oarbot_{color}.urdf.xacro')).toxml()
    robot=ET.fromstring(xml)
    links={node.attrib['name'] for node in robot.findall('link')}
    joints=robot.findall('joint')
    children=[joint.find('child').attrib['link'] for joint in joints]
    assert len(children)==len(set(children))
    assert len(links-set(children))==1
    for joint in joints:
        assert joint.find('parent').attrib['link'] in links
        assert joint.find('child').attrib['link'] in links
    for suffix in ('ft_raw','ft_measurement'):
        assert f'oarbot_{color}/j2n6s300_{suffix}' in links
    for mesh in robot.iter('mesh'):
        filename=mesh.attrib['filename']
        if filename.startswith('package://'):
            package,relative=filename[len('package://'):].split('/',1)
            assert (Path(get_package_share_directory(package))/relative).is_file(), filename
    # MoveIt expansion also validates ros2_control includes and initial positions.
    config=MoveItConfigsBuilder(f'oarbot_{color}',package_name=f'oarbot_{color}_moveit').to_moveit_configs()
    srdf=ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    names={joint.attrib['name'] for joint in joints}
    for group in srdf.findall('group'):
        for joint in group.findall('joint'):
            assert joint.attrib['name'] in names
        for link in group.findall('link'):
            assert link.attrib['name'] in links


def load_launch(filename):
    path=ROOT/'src/oarbot_launch/launch'/filename
    spec=importlib.util.spec_from_file_location(filename.replace('.','_'),path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('filename',[p.name for p in (ROOT/'src/oarbot_launch/launch').glob('*.launch.py')])
def test_launch_descriptions_construct(filename):
    assert load_launch(filename).generate_launch_description() is not None


def test_camera_validation_rejects_unsupported_fps_and_unconfigured_sync():
    module=load_launch('azure_kinect_bringup.launch.py')
    context=LaunchContext()
    context.launch_configurations.update({name: default for name,(default,_) in module.PARAMETERS.items()})
    context.launch_configurations['frame_prefix']='camera/'
    module.validate(context)
    context.launch_configurations.update(fps='30',depth_mode='WFOV_UNBINNED')
    with pytest.raises(ValueError): module.validate(context)
    context.launch_configurations.update(fps='15',depth_mode='NFOV_UNBINNED',wired_sync_mode='1')
    with pytest.raises(ValueError): module.validate(context)
    context.launch_configurations.update(sensor_sn='000123456789',color_exposure_usec='8000')
    module.validate(context)
