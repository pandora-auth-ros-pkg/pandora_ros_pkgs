#!/usr/bin/env python

from __future__ import print_function
import os
import yaml
import argparse
import download_checkmd5

def read_yml(filename):
    with open(filename, 'r') as f:
        yml = yaml.safe_load(f)

    return yml


def main():
    """
    Parses one or more 'extra_files.yaml'files and
    uses the 'download_checkmd5.py' to fetch all files
    """
    parser = argparse.ArgumentParser(description="Parses one or more 'extra_files.yaml' \
    files and uses the 'download_checkmd5.py' to fetch all files.")
    parser.add_argument('file', help="Path to 'extra_files.yaml' files.", nargs='+')
    parser.add_argument('--project-source-dir', help="Package's source directory.")
    parser.add_argument('--project-devel-dir', help="Package's devel directory.")

    args = parser.parse_args()

    for filename in args.file:
        extra_files = read_yml(filename)

        for remote_file in extra_files['files']:
            if not remote_file.has_key('filename'):
                remote_file['filename'] = os.path.basename(remote_file['url'])

            if not remote_file.has_key('destination'):
                # relative to the dir of the 'extra_files.yml'
                dest = os.path.join(os.path.dirname(os.path.abspath(filename)),
                                                remote_file['filename'])
            # if destination is absolute
            elif remote_file['destination'].startswith('/'):
                dest = os.path.join(remote_file['destination'],
                                            remote_file['filename'])
            # project source dir
            elif remote_file['destination'].startswith('$PROJECT_SOURCE_DIR') and \
                    args.project_source_dir is not None:
                dest = os.path.join(args.project_source_dir,
                                                remote_file['filename'])
            # project devel dir
            elif remote_file['destination'].startswith('$PROJECT_DEVEL_DIR') and \
                    args.project_devel_dir is not None:
                dest = os.path.join(args.project_devel_dir,
                                                remote_file['filename'])
            # try to compose something that makes sense
            else:
                dest = os.path.join(os.path.dirname(os.path.abspath(filename)),
                        remote_file['destination'], remote_file['filename'])

            arguments = [remote_file['url'], dest]

            if remote_file.has_key('md5'):
                arguments.append(remote_file['md5'])

            if remote_file.has_key('optional') and remote_file['optional']:
                arguments.append('--ignore_error')

            #print(arguments)
            download_checkmd5.main(arguments)

if __name__ == '__main__':
    main()
