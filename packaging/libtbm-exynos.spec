Name:           libtbm-exynos
Version:        1.2.6
Release:        1
License:        MIT
Summary:        Tizen Buffer Manager - exynos backend
Group:          System/Libraries
Source0:        %{name}-%{version}.tar.gz

BuildRequires:  pkgconfig(libdrm)
BuildRequires:  pkgconfig(libdrm_exynos)
BuildRequires:  pkgconfig(libtbm)
BuildRequires:  pkgconfig(dlog)
BuildRequires:  pkgconfig(libudev)
ExclusiveArch:  %{arm} aarch64

%description
descriptionion: Tizen Buffer manager backend module for exynos

%if 0%{?TZ_SYS_RO_SHARE:1}
# TZ_SYS_RO_SHARE is already defined
%else
%global TZ_SYS_RO_SHARE /usr/share
%endif

%prep
%setup -q

%build

%reconfigure --prefix=%{_prefix} --libdir=%{_libdir}/bufmgr \
%if "%_repository" == "target-circle"
             --enable-align-eight \
	     --enable-cachectrl \
%else
	     --disable-cachectrl \
%endif
            CFLAGS="${CFLAGS} -Wall -Werror" LDFLAGS="${LDFLAGS} -Wl,--hash-style=both -Wl,--as-needed"

make %{?_smp_mflags}

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/%{TZ_SYS_RO_SHARE}/license
cp -af COPYING %{buildroot}/%{TZ_SYS_RO_SHARE}/license/%{name}

%if "%_repository" == "target-circle"
mkdir -p %{buildroot}%{_libdir}/udev/rules.d/
cp -af rules/99-libtbm_exynos.rules %{buildroot}%{_libdir}/udev/rules.d/
%endif

%make_install


%post
if [ -f %{_libdir}/bufmgr/libtbm_default.so ]; then
    rm -rf %{_libdir}/bufmgr/libtbm_default.so
fi
ln -s libtbm_exynos.so %{_libdir}/bufmgr/libtbm_default.so

%postun -p /sbin/ldconfig

%files
%{_libdir}/bufmgr/libtbm_*.so*
%{TZ_SYS_RO_SHARE}/license/%{name}
%if "%_repository" == "target-circle"
%{_libdir}/udev/rules.d/99-libtbm_exynos.rules
%endif
