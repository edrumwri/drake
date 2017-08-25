# -*- mode: python -*-
# vi: set ft=python :

"""
Makes selected VTK headers and precompiled shared libraries available to be
used as a C/C++ dependency. On Ubuntu Trusty and Xenial, a VTK archive is
downloaded and unpacked. On macOS and OS X, VTK must be installed using
Homebrew.

Example:
    WORKSPACE:
        load("//tools:vtk.bzl", "vtk_repository")
        vtk_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:vtkCommonCore"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

VTK_MAJOR_MINOR_VERSION = "8.0"

def _vtk_cc_library(os_name, name, hdrs = None, visibility = None, deps = None,
                    header_only = False, linkopts = []):
    hdr_paths = []

    if hdrs:
        includes = ["include/vtk-{}".format(VTK_MAJOR_MINOR_VERSION)]

        if not visibility:
            visibility = ["//visibility:public"]

        for hdr in hdrs:
            hdr_paths += ["{}/{}".format(includes[0], hdr)]
    else:
        includes = []

        if not visibility:
            visibility = ["//visibility:private"]

    if not deps:
        deps = []

    srcs = []

    if os_name == "mac os x":
        srcs = ["empty.cc"]

        if not header_only:
            linkopts = linkopts + [
                "-L/usr/local/opt/vtk@{}/lib".format(VTK_MAJOR_MINOR_VERSION),
                "-l{}-{}".format(name, VTK_MAJOR_MINOR_VERSION),
            ]
    else:
        if not header_only:
            srcs = ["lib/lib{}-{}.so.1".format(name, VTK_MAJOR_MINOR_VERSION)]

    content = """
cc_library(
    name = "{}",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
    visibility = {},
    deps = {},
)
    """.format(name, srcs, hdr_paths, includes, linkopts, visibility, deps)

    return content

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        repository_ctx.symlink("/usr/local/opt/vtk@{}/include".format(
            VTK_MAJOR_MINOR_VERSION), "include")
        repository_ctx.file("empty.cc", executable = False)

    elif repository_ctx.os.name == "linux":
        sed = repository_ctx.which("sed")
        if sed == None:
            fail("Could NOT determine Linux distribution information" +
                 "('sed' is missing?!)", sed)
        result = repository_ctx.execute([
            sed,
            "-n",
            "/^\(NAME\|VERSION_ID\)=/{s/[^=]*=//;s/\"//g;p}",
            "/etc/os-release"])

        if result.return_code != 0:
            fail("Could NOT determine Linux distribution information",
                 attr = result.stderr)

        distro = [l.strip() for l in result.stdout.strip().split("\n")]
        distro = " ".join(distro)

        if distro == "Ubuntu 14.04":
            archive = "vtk-v8.0.0-qt-4.8.6-trusty-x86_64.tar.gz"
            sha256 = "e5240b6fab2f5d7675d11b77d2014987c5337bb6276e38ab8299a1ab1fee5167"  # noqa
        elif distro == "Ubuntu 16.04":
            archive = "vtk-v8.0.0-qt-5.5.1-xenial-x86_64.tar.gz"
            sha256 = "455edf52f5d7c8d2e8ff6b1e909b6e7c44c61da7922bf8cbe7301a42e9539a3f"  # noqa
        else:
            fail("Linux distribution is NOT supported", attr = distro)

        url = "https://d2mbb5ninhlpdu.cloudfront.net/vtk/{}".format(archive)
        root_path = repository_ctx.path("")

        repository_ctx.download_and_extract(url, root_path, sha256 = sha256)

    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    # Note that we only create library targets for enough of VTK to support
    # those used directly or indirectly by Drake.

    # TODO(jamiesnape): Create a script to help generate the targets.

    # To see what the VTK module dependencies are, you can inspect VTK's source
    # tree. For example, for vtkIOXML and vtkIOXMLParser:
    #   VTK/IO/XML/module.cmake
    #   VTK/IO/XMLParser/module.cmake

    file_content = _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonColor",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonComputationalGeometry",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonCore",
        hdrs = [
            "vtkABI.h",
            "vtkAbstractArray.h",
            "vtkAOSDataArrayTemplate.h",
            "vtkAOSDataArrayTemplate.txx",
            "vtkArrayIterator.h",
            "vtkArrayIteratorTemplate.h",
            "vtkArrayIteratorTemplate.txx",
            "vtkAtomic.h",
            "vtkAtomicTypeConcepts.h",
            "vtkAtomicTypes.h",
            "vtkAutoInit.h",
            "vtkBuffer.h",
            "vtkCollection.h",
            "vtkCommonCoreModule.h",
            "vtkConfigure.h",
            "vtkDataArray.h",
            "vtkDebugLeaksManager.h",
            "vtkGenericDataArray.h",
            "vtkGenericDataArray.txx",
            "vtkGenericDataArrayLookupHelper.h",
            "vtkIdList.h",
            "vtkIdTypeArray.h",
            "vtkIndent.h",
            "vtkIntArray.h",
            "vtkIOStream.h",
            "vtkMath.h",
            "vtkMathConfigure.h",
            "vtkNew.h",
            "vtkObject.h",
            "vtkObjectBase.h",
            "vtkObjectFactory.h",
            "vtkOStreamWrapper.h",
            "vtkOStrStreamWrapper.h",
            "vtkPoints.h",
            "vtkSetGet.h",
            "vtkSmartPointer.h",
            "vtkSmartPointerBase.h",
            "vtkStdString.h",
            "vtkSystemIncludes.h",
            "vtkTimeStamp.h",
            "vtkType.h",
            "vtkTypeTraits.h",
            "vtkUnicodeString.h",
            "vtkUnsignedCharArray.h",
            "vtkVariant.h",
            "vtkVariantCast.h",
            "vtkVariantInlineOperators.h",
            "vtkVersion.h",
            "vtkVersionMacros.h",
            "vtkWeakPointerBase.h",
            "vtkWin32Header.h",
            "vtkWindow.h",
            "vtkWrappingHints.h",
        ],
        deps = [
            ":vtkkwiml",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonDataModel",
        hdrs = [
            "vtkAbstractCellLinks.h",
            "vtkCell.h",
            "vtkCellArray.h",
            "vtkCellData.h",
            "vtkCellLinks.h",
            "vtkCellType.h",
            "vtkCellTypes.h",
            "vtkCommonDataModelModule.h",
            "vtkDataObject.h",
            "vtkDataSet.h",
            "vtkDataSetAttributes.h",
            "vtkFieldData.h",
            "vtkImageData.h",
            "vtkPointSet.h",
            "vtkPolyData.h",
            "vtkRect.h",
            "vtkStructuredData.h",
            "vtkVector.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonExecutionModel",
        hdrs = [
            "vtkAlgorithm.h",
            "vtkCommonExecutionModelModule.h",
            "vtkImageAlgorithm.h",
            "vtkPolyDataAlgorithm.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonMisc",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonMath",
        hdrs = [
            "vtkCommonMathModule.h",
            "vtkMatrix4x4.h",
            "vtkTuple.h",
        ],
        deps = [":vtkCommonCore"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonMisc",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonSystem",
        deps = [":vtkCommonCore"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonTransforms",
        hdrs = [
            "vtkAbstractTransform.h",
            "vtkCommonTransformsModule.h",
            "vtkHomogeneousTransform.h",
            "vtkLinearTransform.h",
            "vtkTransform.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkDICOMParser",
        deps = [":vtksys"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersCore",
        hdrs = [
            "vtkCleanPolyData.h",
            "vtkFiltersCoreModule.h",
        ],
        visibility = ["//visibility:private"],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersGeometry",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersGeneral",
        hdrs = [
            "vtkFiltersGeneralModule.h",
            "vtkTransformPolyDataFilter.h",
        ],
        deps = [
            ":vtkCommonComputationalGeometry",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkFiltersCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersSources",
        hdrs = [
            "vtkCubeSource.h",
            "vtkCylinderSource.h",
            "vtkFiltersSourcesModule.h",
            "vtkPlaneSource.h",
            "vtkSphereSource.h",
        ],
        deps = [
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkFiltersCore",
            ":vtkFiltersGeneral",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOCore",
        hdrs = [
            "vtkAbstractPolyDataReader.h",
            "vtkIOCoreModule.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            ":vtklz4",
        ],
    )

    # See: VTK/IO/XMLParser/{*.h,module.cmake}
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOXMLParser",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkexpat",
            ":vtkIOCore",
            ":vtksys",
        ],
    )

    # See: VTK/IO/XML/{*.h,module.cmake}
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOXML",
        hdrs = [
            "vtkIOXMLModule.h",
            "vtkXMLDataReader.h",
            "vtkXMLPolyDataReader.h",
            "vtkXMLReader.h",
            "vtkXMLUnstructuredDataReader.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
            ":vtkIOXMLParser",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOGeometry",
        hdrs = [
            "vtkIOGeometryModule.h",
            "vtkOBJReader.h",
        ],
        deps = [
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
            ":vtkIOLegacy",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOImage",
        hdrs = [
            "vtkImageExport.h",
            "vtkImageReader2.h",
            "vtkIOImageModule.h",
            "vtkPNGReader.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            ":vtkDICOMParser",
            ":vtkmetaio",
            "@libpng",
            "@zlib",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOImport",
        hdrs = [
            "vtkImporter.h",
            "vtkIOImportModule.h",
            "vtkOBJImporter.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkRenderingCore",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOLegacy",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingCore",
        hdrs = [
            "vtkAbstractMapper.h",
            "vtkAbstractMapper3D.h",
            "vtkActor.h",
            "vtkActorCollection.h",
            "vtkCamera.h",
            "vtkMapper.h",
            "vtkPolyDataMapper.h",
            "vtkProp.h",
            "vtkProp3D.h",
            "vtkPropCollection.h",
            "vtkProperty.h",
            "vtkRenderer.h",
            "vtkRenderingCoreModule.h",
            "vtkRenderWindow.h",
            "vtkTexture.h",
            "vtkViewport.h",
            "vtkVolume.h",
            "vtkVolumeCollection.h",
            "vtkWindowToImageFilter.h",
        ],
        deps = [
            ":vtkCommonColor",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkFiltersCore",
            ":vtkFiltersGeometry",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingOpenGL2",
        visibility = ["//visibility:public"],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkRenderingCore",
            ":vtkglew",
        ],
    )

    # The packaged version of VTK (both Ubuntu and Mac) uses the system version
    # of expat, and do not include `vtkexpat` as a shared library.
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkexpat",
        linkopts = [
            "-lexpat",
        ],
        header_only = True,
    )

    if repository_ctx.os.name == "mac os x":
        file_content += """
cc_library(
    name = "vtkglew",
    srcs = ["empty.cc"],
    linkopts = [
        "-L/usr/local/opt/glew/lib",
        "-lGLEW",
    ],
    visibility = ["//visibility:private"],
)
        """
    else:
        file_content += _vtk_cc_library(repository_ctx.os.name, "vtkglew")

    file_content += _vtk_cc_library(
        repository_ctx.os.name, "vtkkwiml",
        hdrs = [
            "vtk_kwiml.h",
            "vtkkwiml/abi.h",
            "vtkkwiml/int.h",
        ],
        visibility = ["//visibility:private"],
        header_only = True,
    )

    if repository_ctx.os.name == "mac os x":
        file_content += """
cc_library(
    name = "vtklz4",
    srcs = ["empty.cc"],
    linkopts = [
        "-L/usr/local/opt/lz4/lib",
        "-llz4",
    ],
    visibility = ["//visibility:private"],
)
        """
    else:
        file_content += _vtk_cc_library(repository_ctx.os.name, "vtklz4")

    file_content += _vtk_cc_library(repository_ctx.os.name, "vtkmetaio",
                                    deps = ["@zlib"])

    file_content += _vtk_cc_library(repository_ctx.os.name, "vtksys")

    # Glob all files for the data dependency of drake-visualizer.
    file_content += """
filegroup(
    name = "vtk",
    srcs = glob(["**/*"], exclude=["BUILD", "WORKSPACE"]),
    visibility = ["//visibility:public"],
)
"""

    if repository_ctx.os.name == "mac os x":
        # Use Homebrew VTK.
        files_to_install = []
    else:
        # Install all files.
        files_to_install = [":vtk"]

    file_content += """
load("@drake//tools:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = {},
    visibility = ["//visibility:public"],
)
""".format(files_to_install)

    repository_ctx.file("BUILD", content = file_content, executable = False)

vtk_repository = repository_rule(implementation = _impl)
