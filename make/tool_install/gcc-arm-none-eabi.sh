if [ "$uname" = Linux ]
then
	url_ext="x86_64-linux.tar.bz2"
    tool_md5="2383e4eb4ea23f248d33adc70dc3227e"
elif [ "$uname" = Darwin ]
then
	url_ext="mac.tar.bz2"
    tool_md5="7f2a7b7b23797302a9d6182c6e482449"
elif [ "$uname" = Windows ]
then
	url_ext="win32.zip"
    tool_md5="2bc8f0c4c4659f8259c8176223eeafc1"
	depends=(7z)
fi

pkgver=10.3-2021.10
_pkgver=${pkgver//_/-}
_pkgvershort=${_pkgver%-*}
_pkgvershort=${_pkgvershort/-q/q}

tool_url="https://developer.arm.com/-/media/Files/downloads/gnu-rm/${pkgver}/${tool}-${pkgver}-${url_ext}"
#tool_install_name="${tool}-${_pkgvershort/./_}"
tool_install_name="${tool}-${pkgver}"
if [ "$uname" = Windows ]
then
	tool_extract_dir=$tools_dir/$tool_install_name
fi

bin_subdir=bin

function validate_target { true; }
