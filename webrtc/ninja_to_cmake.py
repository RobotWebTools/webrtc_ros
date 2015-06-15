#!/usr/bin/env python

import copy
import os
import shutil
import re
import argparse
import subprocess


class Build(object):
    def __init__(self, rule, outputs, inputs):
        self.rule = rule
        self.outputs = outputs
        self.inputs = inputs

def parse_file(filename):
    builds = []
    definitions = {}
    def process_line(line):
        definition_match_result = re.match('^(\w+) = (.*)$', line)
        if definition_match_result:
            name = definition_match_result.group(1)
            value = definition_match_result.group(2)
            definitions[name] = value
        elif line.startswith('build '):
            statement_components = line[len('build '):].split(':', 1)
            build_outputs = statement_components[0].split(' ')
            statement_second_half = statement_components[1].lstrip().split(' ')
            build_rule = statement_second_half[0]
            build_inputs = []
            statement_second_half_iter = iter(statement_second_half[1:])
            try:
                while True:
                    component = statement_second_half_iter.next()
                    if component == '|': # Implicit dependencies
                        statement_second_half_iter.next()
                    elif component == '||': # Order-only dependencies
                        statement_second_half_iter.next()
                    else:
                        build_inputs.append(component)
            except StopIteration:
                pass

            builds.append(Build(build_rule, build_outputs, build_inputs))
        else:
            pass
            print 'Could not process line: ' + line


    with open(filename) as f:
        logical_lines = []
        partial_line = ''
        for line in f:
            line = line.split('#', 1)[0] # remove comments
            if len(line.strip()) == 0:
                continue
            line = line.lstrip()
            if line.endswith('$\n'):
                partial_line += line[:-2]
            else:
                process_line(partial_line + line[:-1])
                partial_line = ''
        return (builds, definitions)


parser = argparse.ArgumentParser(description='Copy files/configuration from the ninja build file')
parser.add_argument('build_root')
subparsers = parser.add_subparsers(dest='action')

copy_libs_parser = subparsers.add_parser('copy-libs')
copy_libs_parser.add_argument('library_output_dir')

copy_libs_parser = subparsers.add_parser('copy-tools')
copy_libs_parser.add_argument('tool_output_dir')

built_libs_parser = subparsers.add_parser('list-built-libs')
built_libs_parser.add_argument('library_output_dir')

install_includes_parser = subparsers.add_parser('install-includes')
install_includes_parser.add_argument('include_install_dir')

generate_cmake_extras_parser = subparsers.add_parser('generate-cmake-extras')
generate_cmake_extras_parser.add_argument('output_file')
generate_cmake_extras_parser.add_argument('library_output_dir')

args = parser.parse_args()

builds, definitions = parse_file(os.path.join(args.build_root, 'bare_executable.ninja'))

for build in builds:
    if build.rule == 'link' and build.outputs == ['bare_executable']:
        built_lib_paths = copy.copy(build.inputs)
        built_lib_paths.remove('bare_executable.bare.o')
        built_lib_paths = filter(lambda f: not f.endswith('.stamp'), built_lib_paths)


built_libs = []
for lib in built_lib_paths:
    old_name = os.path.basename(lib)
    if not old_name.startswith('lib'):
        raise Exception('library file name does not begin with lib')

    new_name = 'libwebrtc_'+old_name[3:]
    link_name = os.path.splitext(new_name)[0][3:]
    built_libs.append((os.path.join(args.build_root, lib), new_name, link_name))

src_include_dirs = map(lambda include: os.path.normpath(os.path.join(args.build_root, re.sub('^-I', '', include))), definitions['includes'].split())
src_include_dirs = filter(lambda path: not path.endswith('third_party/webrtc'), src_include_dirs) # this path is added but does not exist in this source configuration


if args.action == 'copy-libs':
    if not os.path.isdir(args.library_output_dir):
        os.makedirs(args.library_output_dir)
    for lib in built_libs:
        input_lib = lib[0]
        output_lib = os.path.join(args.library_output_dir, lib[1])
        input_time = os.path.getmtime(input_lib)
        output_time = 0 if not os.path.isfile(output_lib) else os.path.getmtime(output_lib)
        # need a fudge factor because copystat below does not appear to be able to copy the exect modification time
        # potentially related to http://stackoverflow.com/questions/17086426/file-modification-times-not-equal-after-calling-shutil-copystatfile1-file2-un
        if output_time < input_time - 1e-6:
            with open(input_lib, 'r') as input_f:
                if input_f.readline() == '!<thin>\n': # if is a thin archive create a normal one, otherwise just copy the archive
                    ar = 'ar' # TODO should eventually import ar tool name from build.ninja
                    object_files = subprocess.check_output([ar, 't', input_lib]).split()
                    if os.path.isfile(output_lib):
                        os.remove(output_lib)
                    subprocess.check_call([ar, 'rs', output_lib] + object_files, cwd=os.path.dirname(input_lib))
                    shutil.copystat(input_lib, output_lib)
                else:
                    shutil.copy(input_lib, output_lib)

elif args.action == 'copy-tools':
    if not os.path.isdir(args.tool_output_dir):
        os.makedirs(args.tool_output_dir)
    for tool in ['stunserver', 'turnserver', 'relayserver']:
        input_tool = os.path.join(args.build_root, tool)
        output_tool = os.path.join(args.tool_output_dir, tool)
        input_time = os.path.getmtime(input_tool)
        output_time = 0 if not os.path.isfile(output_tool) else os.path.getmtime(output_tool)
        if output_time < input_time:
            shutil.copy(input_tool, output_tool)

elif args.action == 'list-built-libs':
    print ';'.join(os.path.join(args.library_output_dir, lib[1]) for lib in built_libs)

elif args.action == 'install-includes':
    print 'Installing includes'
    seen = set()
    dirs = reversed([include_dir for include_dir in src_include_dirs if not (include_dir in seen or seen.add(include_dir))])
    for include_dir in dirs:
        for root, dirs, files in os.walk(include_dir, followlinks=True):
            dest_dir = os.path.join(args.include_install_dir, os.path.relpath(root, include_dir))
            if not os.path.isdir(dest_dir):
                os.makedirs(dest_dir)
            for filename in files:
                if filename.endswith('.h'):
                    input_file = os.path.join(root, filename)
                    output_file = os.path.join(args.include_install_dir, os.path.relpath(input_file, include_dir))
                    #print 'Installing: ' + output_file
                    shutil.copyfile(input_file, output_file)

elif args.action == 'generate-cmake-extras':
    with open(args.output_file, 'w') as f:
        def write_set(name, value):
            if type(value) is str:
                f.write('set(' + name + ' "' + value + '")\n')
            else:
                f.write('set(' + name + '\n')
                for item in value:
                    f.write('  ' + str(item) + '\n')
                f.write(')\n')

        libs = map(lambda lib: os.path.join(args.library_output_dir, lib[1]), built_libs) + definitions['libs'].split()
        write_set('__WEBRTC_LIBRARIES', libs)
        write_set('WEBRTC_LIBRARIES', '${__WEBRTC_LIBRARIES} ${__WEBRTC_LIBRARIES} ${__WEBRTC_LIBRARIES} ${__WEBRTC_LIBRARIES}')


        cflags = definitions['cflags'].split()
        cflags.remove('-fvisibility=hidden') # we want to export symbols

        cflags_c = definitions['cflags_c'].split()

        cflags_cc = definitions['cflags_cc'].split()
        cflags_cc.remove('-fno-exceptions') # we use exceptions

        ldflags = definitions['ldflags'].split()

        defines = definitions['defines'].split()

        write_set('WEBRTC_INCLUDE_DIRS', src_include_dirs) # list
        write_set('WEBRTC_COMPILE_OPTIONS', cflags + cflags_c + cflags_cc) # list
        write_set('WEBRTC_LINK_FLAGS', ' '.join(ldflags)) # string
        write_set('WEBRTC_COMPILE_DEFINITIONS', map(lambda d: re.sub('^-D', '', d), defines)) # list

else:
    print "Unknown action"
