Name:           libtbm-vc4
Version:        0.0.5
Release:        1
License:        MIT
Summary:        Tizen Buffer Manager - vc4 backend
Group:          System/Libraries
Source0:        %{name}-%{version}.tar.gz

BuildRequires:  pkgconfig(libdrm)
BuildRequires:  pkgconfig(libdrm_vc4)
BuildRequires:  pkgconfig(libtbm)
BuildRequires:  pkgconfig(dlog)
BuildRequires:  pkgconfig(libudev)
ExclusiveArch:  %{arm} aarch64

%description
descriptionion: Tizen Buffer manager backend module for vc4

%if 0%{?TZ_SYS_RO_SHARE:1}
# TZ_SYS_RO_SHARE is already defined
%else
%global TZ_SYS_RO_SHARE /usr/share
%endif

%prep
%setup -q

%build

%reconfigure --prefix=%{_prefix} --libdir=%{_libdir}/bufmgr \
			--disable-align-eight \
			--disable-cachectrl \
			CFLAGS="${CFLAGS} -Wall -Werror" LDFLAGS="${LDFLAGS} -Wl,--hash-style=both -Wl,--as-needed"

make %{?_smp_mflags}

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/%{TZ_SYS_RO_SHARE}/license
cp -af COPYING %{buildroot}/%{TZ_SYS_RO_SHARE}/license/%{name}

# make rule for tgl
mkdir -p %{buildroot}%{_libdir}/udev/rules.d/
cp -af rules/99-libtbm-vc4.rules %{buildroot}%{_libdir}/udev/rules.d/

%make_install

%post
if [ -f %{_libdir}/bufmgr/libtbm_default.so ]; then
    rm -rf %{_libdir}/bufmgr/libtbm_default.so
fi
ln -s libtbm-vc4.so %{_libdir}/bufmgr/libtbm_default.so

%postun -p /sbin/ldconfig

%files
%{_libdir}/bufmgr/libtbm-*.so*
%{TZ_SYS_RO_SHARE}/license/%{name}
%{_libdir}/udev/rules.d/99-libtbm-vc4.rules
