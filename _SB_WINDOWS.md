### Command:
```
make PYTHON=python2 build_sdk_install
```

Problem:
```
Failed to verify D:/CODE/CarController/librepilot_win/downloads/gcc-arm-none-eabi-10.3-2021.10-win32.zip
make: *** [D:/CODE/CarController/librepilot_win/make/tools.mk:69: gcc-arm-none-eabi_install] Error 1
```

Fix: (tool_install.sh)
```
		# _SB_ '| cut -d' ' -f1' added , because md5sum returns eb7c9d1e131595525630f7dca8f13f09 *- (with star)
		if [[ "${tool_md5:-}" != "$(cd "$downloads_dir" && md5sum <"$downloaded_file" | cut -d' ' -f1)" ]]
```

### Command:
```
make PYTHON=python2 gcs
```

