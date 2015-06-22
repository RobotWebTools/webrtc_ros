#!/usr/bin/env python

import os
import subprocess
import sys
import urllib2
import shutil
import tarfile
import base64
import subprocess

dev_null = open(os.devnull, "w")

PACKAGE_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(PACKAGE_SRC_DIR, 'src')

CHROMIUM_BASE_ARCHIVE_URL="https://chromium.googlesource.com/chromium/src/+archive/"
CHROMIUM_ARCHIVE_FILETYPE=".tar.gz"

CHROMIUM_BASE_FILE_URL="https://chromium.googlesource.com/chromium/src/+/"
CHROMIUM_FILE_URL_SUFFIX="?format=TEXT"

CHROMIUM_SRC_DIRS = [
    "testing",
    "build",
    "tools/clang",
    "tools/generate_shim_headers",
    "tools/protoc_wrapper",
    "third_party/jsoncpp",
    "third_party/usrsctp",
    "third_party/expat",
    "third_party/protobuf",
    "third_party/opus",
    "third_party/yasm",
    "net/third_party/nss",
    "third_party/binutils",
]
CHROMIUM_SRC_FILES = [
    "DEPS",
]

CHROMIUM_DEPS = [
    "third_party/libyuv",
    "third_party/libsrtp",
    "third_party/libvpx",
    "third_party/libjpeg_turbo",
    "third_party/opus/src",
    "third_party/openmax_dl",
    "third_party/yasm/source/patched-yasm",
    "third_party/usrsctp/usrsctplib",
    "tools/gyp",
    "third_party/jsoncpp/source/src/lib_json",
    "third_party/jsoncpp/source/include",
]

CHROMIUM_HOOKS = [
    "binutils",
]

SCRIPT_VERSION = 1

def execDEPS(f):
    DEPS_vars = {}

    def Var(name):
        return DEPS_vars["vars"][name]

    DEPS_vars["Var"] = Var

    execfile(f, DEPS_vars)
    return DEPS_vars




def main():
    CHROMIUM_DIR = os.path.join(ROOT_DIR, 'chromium')
    CHROMIUM_SRC_DIR = os.path.join(CHROMIUM_DIR, 'src')

    webrtc_DEPS_file = os.path.join(ROOT_DIR, 'DEPS')
    target_revision = execDEPS(webrtc_DEPS_file)["vars"]["chromium_revision"]

    print "Chromium Target Revision: " + target_revision

    for name in CHROMIUM_SRC_DIRS:
        dep_dir = os.path.join(CHROMIUM_SRC_DIR, name)
        flag_file = os.path.join(dep_dir, '.last_sync')
        flag_file_content = '\n'.join([
            str(SCRIPT_VERSION),
            target_revision
        ])

        up_to_date = False
        if os.path.isdir(dep_dir) and os.path.isfile(flag_file):
            with open(flag_file, 'r') as f:
                if f.read() == flag_file_content:
                    up_to_date = True

        if up_to_date:
            print name + " is up to date at: " + target_revision
        else:
            archive_url = CHROMIUM_BASE_ARCHIVE_URL + target_revision + "/" + name + CHROMIUM_ARCHIVE_FILETYPE
            archive_file = os.path.join(dep_dir, "archive"+CHROMIUM_ARCHIVE_FILETYPE)
            if os.path.isdir(dep_dir):
                shutil.rmtree(dep_dir)
            os.makedirs(dep_dir)

            # Download archive
            with open(archive_file, 'w') as f:
                print "Getting: " + archive_url
                response = urllib2.urlopen(archive_url)
                f.write(response.read())

            # Extract archive
            tar = tarfile.open(archive_file)
            tar.extractall(dep_dir)
            tar.close()

            # Write flag file
            with open(flag_file, 'w') as f:
                f.write(flag_file_content)

    for name in CHROMIUM_SRC_FILES:
        dep_file = os.path.join(CHROMIUM_SRC_DIR, name)
        flag_file = os.path.join(os.path.dirname(dep_file), '.'+name+'_last_sync')
        flag_file_content = '\n'.join([
            str(SCRIPT_VERSION),
            target_revision
        ])

        up_to_date = False
        if os.path.isfile(dep_file) and os.path.isfile(flag_file):
            with open(flag_file, 'r') as f:
                if f.read() == flag_file_content:
                    up_to_date = True

        if up_to_date:
            print name + " is up to date at: " + target_revision
        else:
            archive_url = CHROMIUM_BASE_FILE_URL + target_revision + "/" + name + CHROMIUM_FILE_URL_SUFFIX

            parent_dir = os.path.dirname(dep_file)
            if not os.path.isdir(parent_dir):
                os.makedirs(parent_dir)

            # Download file
            with open(dep_file, 'w') as f:
                print "Getting: " + archive_url
                response = urllib2.urlopen(archive_url)
                f.write(base64.b64decode(response.read()))

            # Write flag file
            with open(flag_file, 'w') as f:
                f.write(flag_file_content)


    chromium_DEPS_file = os.path.join(CHROMIUM_SRC_DIR, "DEPS")
    chromium_DEPS_vars = execDEPS(chromium_DEPS_file)
    def lookupChromiumDep(name):
        full_name = os.path.join("src", name)
        dep_dicts = [chromium_DEPS_vars["deps_os"]["unix"], chromium_DEPS_vars["deps"]]
        for dep_dict in dep_dicts:
            if full_name in dep_dict:
                return dep_dict[full_name]
        return None

    def lookupChromiumHook(name):
        for hook in chromium_DEPS_vars["hooks"]:
            if hook['name'] == name:
                return hook
        return None

    for name in CHROMIUM_DEPS:
        dep = lookupChromiumDep(name)
        if dep is None:
            print "Could not find dep: " + name
            return 1
        dep_repo, dep_revision = dep.split('@')

        dep_destination = os.path.join(CHROMIUM_SRC_DIR, name)

        if not os.path.isdir(dep_destination):
            print name + " does not exist, cloning: " + dep_repo
            subprocess.check_call(["git", "clone", "-q", dep_repo, dep_destination])
            subprocess.check_call(["git", "config", "remote.origin.fetch",
                                   "+refs/branch-heads/*:refs/remotes/branch-heads/*",
                                   "^\\+refs/branch-heads/\\*:.*$"], cwd=dep_destination)
            subprocess.check_call(["git", "config", "remote.origin.fetch",
                                   "+refs/tags/*:refs/tags/*",
                                   "^\\+refs/tags/\\*:.*$"], cwd=dep_destination)
            subprocess.check_call(["git", "fetch", "-q", "--all"], cwd=dep_destination)

        current_revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=dep_destination).strip()
        if current_revision != dep_revision:
            if subprocess.call(["git", "show", dep_revision], cwd=dep_destination, stdout=dev_null, stderr=dev_null) != 0: # will fail if commit does not exist
                print "Could not find requested commit: "+dep_revision+", fetching"
                subprocess.check_call(["git", "fetch", "-q", "--all"], cwd=dep_destination)
            print "Checking out: " + dep_revision + ' for ' + name
            subprocess.check_call(["git", "checkout", dep_revision], cwd=dep_destination)
        else:
            print name + " is up to date at " + current_revision


    # handle using system libraries
    replace_gyp_files_location = os.path.join(CHROMIUM_SRC_DIR, "build", "linux", "unbundle")
    sys.path.append(replace_gyp_files_location)
    SYSTEM_REPLACEMENTS = __import__("replace_gyp_files").REPLACEMENTS

    defines = os.environ['GYP_DEFINES']
    # ensure file to be replace exist
    for flag, path in SYSTEM_REPLACEMENTS.items():
        if '%s=1' % flag in defines:
            replaced_file = os.path.join(CHROMIUM_SRC_DIR, path)
            if not os.path.isfile(replaced_file):
                if not os.path.isdir(os.path.dirname(replaced_file)):
                    os.makedirs(os.path.dirname(replaced_file))
                with open(replaced_file, 'w') as f:
                    f.write("")

    command = ["python", "replace_gyp_files.py"]
    command.extend(["-D"+define for define in defines.split()])
    subprocess.check_call(command, cwd=replace_gyp_files_location)

    hook_wd = CHROMIUM_DIR
    for name in CHROMIUM_HOOKS:
        hook = lookupChromiumHook(name)
        if hook is None:
            print "Could not find hook: " + name
            return 1
        print "Running hook: " + name
        subprocess.check_call(hook['action'], cwd=hook_wd)

    return 0

if __name__ == '__main__':
  sys.exit(main())
