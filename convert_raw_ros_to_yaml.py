import yaml # pip install pyyaml
import os
import io

def convert_raw_ros_checkerboard_to_yaml(raw_ros_file, yaml_template_file):
    
    with open(yaml_template_file, 'r') as stream:
        yaml_data = yaml.safe_load(stream)
    
    camera_id = os.path.basename(raw_ros_file).split('.')[0]
    with open(raw_ros_file) as ros_data:
        ros_data_lines = ros_data.readlines()

    def extract_data(ros_data_lines, key, n_lines):
        data = ''
        for i, line in enumerate(ros_data_lines):
            if key in line:
                for l in range(n_lines):
                    data += ros_data_lines[i+1+l].rstrip()+ ' '
        data = data.strip()
        data_num = [float(d) for d in data.split(' ')]
        return data_num

    yaml_data['image_width'] = int(extract_data(ros_data_lines, 'width', 1)[0])
    yaml_data['image_height'] = int(extract_data(ros_data_lines, 'height', 1)[0])
    yaml_data['camera_name'] = camera_basename+camera_id
    yaml_data['camera_matrix'] = {'rows': 3, 'cols': 3, 'data': extract_data(ros_data_lines, 'camera matrix', 3)}
    yaml_data['distortion_coefficients'] = {'rows': 1, 'cols': 5, 'data': extract_data(ros_data_lines, 'distortion', 1)}
    yaml_data['rectification_matrix'] = {'rows': 3, 'cols': 3, 'data': extract_data(ros_data_lines, 'rectification', 3)}
    yaml_data['projection_matrix'] = {'rows': 3, 'cols': 4, 'data': extract_data(ros_data_lines, 'projection', 3)}

    fname = camera_basename+camera_id+'.yaml'
    fname_with_path = os.path.join(dest_dir, fname)

    with io.open(fname_with_path, 'w', encoding='utf8') as outfile:
        yaml.dump(yaml_data, outfile, default_flow_style=False, allow_unicode=True)

    print('Wrote yaml file to: ' + fname_with_path)

if __name__ == '__main__':

    yaml_template_file = "example_camera.yaml"
    source_dir = 'test_data'
    dest_dir = 'test_data_yamls'
    camera_basename = 'Basler-'

    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    raw_ros_files = os.listdir(source_dir)
    for raw_ros_file in raw_ros_files:
        raw_ros_file_with_path = os.path.join(source_dir, raw_ros_file)
        convert_raw_ros_checkerboard_to_yaml(raw_ros_file_with_path, yaml_template_file)
