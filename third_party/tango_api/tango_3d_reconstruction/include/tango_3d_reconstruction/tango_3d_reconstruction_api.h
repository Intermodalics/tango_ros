// Copyright 2016 Google Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef TANGO_3D_RECONSTRUCTION_API_TANGO_3D_RECONSTRUCTION_API_H_
#define TANGO_3D_RECONSTRUCTION_API_TANGO_3D_RECONSTRUCTION_API_H_

#include <stdbool.h>
#include <stdint.h>

/// @file tango_3d_reconstruction_api.h
/// @brief File containing Project Tango 3D Reconstruction C API

#ifdef __cplusplus
extern "C" {
#endif

/// @defgroup Enums Enumerated Types
/// @brief Enums for camera, configuration, and error types.
/// @{

/// @brief 3D Reconstruction Error types.
/// Errors less than 0 should be dealt with by the program.
/// Success is denoted by <code>TANGO_3DR_SUCCESS = 0</code>.
typedef enum {
  /// This error code denotes some sort of hard error occurred.
  TANGO_3DR_ERROR = -3,
  /// There is not enough space in a provided buffer.
  TANGO_3DR_INSUFFICIENT_SPACE = -2,
  /// The input argument is invalid.
  TANGO_3DR_INVALID = -1,
  /// This code indicates success.
  TANGO_3DR_SUCCESS = 0
} Tango3DR_Status;

/// Tango 3DR Image Formats
///
/// Equivalent to those found in Android core/system/include/system/graphics.h.
/// See TangoImageBuffer for a format description.
typedef enum {
  TANGO_3DR_HAL_PIXEL_FORMAT_RGBA_8888 = 1,       ///< RGBA 8888
  TANGO_3DR_HAL_PIXEL_FORMAT_RGB_888 = 3,         ///< RGB 888
  TANGO_3DR_HAL_PIXEL_FORMAT_YV12 = 0x32315659,   ///< YV12
  TANGO_3DR_HAL_PIXEL_FORMAT_YCrCb_420_SP = 0x11  ///< NV21
} Tango3DR_ImageFormatType;

/// Tango 3DR Camera Calibration types.
typedef enum {
  TANGO_3DR_CALIBRATION_UNKNOWN = 0,
  /// The FOV camera model described in
  /// <a href="http://scholar.google.com/scholar?cluster=13508836606423559694">
  /// Straight lines have to be straight</a>.
  TANGO_3DR_CALIBRATION_EQUIDISTANT = 1,
  /// Brown's distortion model, with the parameter vector representing
  /// distortion as [k1, k2].
  /// <a
  /// href="http://en.wikipedia.org/wiki/Distortion_%28optics%28#Software_correction">
  TANGO_3DR_CALIBRATION_POLYNOMIAL_2_PARAMETERS = 2,
  /// Brown's distortion model, with the parameter vector representing the
  /// distortion as [k1, k2, k3].
  TANGO_3DR_CALIBRATION_POLYNOMIAL_3_PARAMETERS = 3,
  /// Brown's distortion model, with the parameter vector representing the
  /// distortion as [k1, k2, p1, p2, k3].
  TANGO_3DR_CALIBRATION_POLYNOMIAL_5_PARAMETERS = 4,
} Tango3DR_TangoCalibrationType;

/// @brief Tango 3DR configuration enumerations.
typedef enum {
  /// Configuration for Tango3DR_create.
  TANGO_3DR_CONFIG_CONTEXT = 0,
  /// Texturing configuration for Tango3DR_textureMeshFromDataset.
  // TODO(idryanov): Call this TANGO_3DR_CONFIG_TEXTURING_CONTEXT
  TANGO_3DR_CONFIG_TEXTURING = 1
} Tango3DR_ConfigType;

/// @brief Enumerates the available texturing backends.
typedef enum {
  /// Default CPU texturing pipeline.
  TANGO_3DR_CPU_TEXTURING = 0,
  /// OpenGL texturing pipeline, trading some quality for performance.
  TANGO_3DR_GL_TEXTURING = 1,
} Tango3DR_TexturingBackend;

/// @brief 3D Reconstruction update algorithm types.
/// Determines the algorithm used to update the reconstruction during the
/// Tango3DR_update call. The default value is TANGO_3DR_TRAVERSAL_UPDATE.
typedef enum {
  /// Associates voxels with depth readings by traversing (raycasting) forward
  /// from the camera to the observed depth. Results in slightly higher
  /// reconstruction quality. Can be significantly slower, especially on
  /// updates with a high number of depth points.
  TANGO_3DR_TRAVERSAL_UPDATE = 0,
  /// Associates voxels with depth readings by projecting voxels into a depth
  /// image plane using a projection matrix. Requires that the depth camera
  /// calibration has been set using the Tango3DR_setDepthCalibrtion method.
  /// Results in slightly lower reconstruction quality. Under this mode, the
  /// speed of updates is independent of the number of depth points.
  TANGO_3DR_PROJECTIVE_UPDATE = 1,
} Tango3DR_UpdateMethod;

/**@} */

/// @defgroup MeshBuildingTypes Mesh Building Types
/// @brief Types used when building meshes.
/// @{

/// This provides a handle to a Tango 3D reconstruction server.
typedef void* Tango3DR_Context;

/// This provides a handle to a Tango 3D texturing context.
typedef void* Tango3DR_TexturingContext;

/// This defines a handle to a Tango3DR_ConfigH; key/value pairs can only be
/// accessed through API calls.
typedef void* Tango3DR_ConfigH;

/// This defines an opaque handle to a trajectory, representing a path
/// traveled over time.
typedef void* Tango3DR_TrajectoryH;

/// An array of three floats, commonly a 3D position or normal.
typedef float Tango3DR_Vector3[3];

/// An array of four floats, commonly a point cloud point with
/// confidence.
typedef float Tango3DR_Vector4[4];

/// An array of three 32-bit integers describing a single triangle in
/// an index buffer.
typedef uint32_t Tango3DR_Face[3];

/// An array of four 8-bit integers describing a color in RGBA order.
typedef uint8_t Tango3DR_Color[4];

/// An array of three integers describing a specific index in the 3D
/// Reconstruction volume grid.
typedef int Tango3DR_GridIndex[3];

/// An array of two integers describing a specific location in the 2D.
typedef int Tango3DR_GridIndex2D[2];

/// An array of two floats describing a texture coordinate in UV order.
typedef float Tango3DR_TexCoord[2];

/// The Tango3DR_CameraCalibration struct contains intrinsic
/// parameters for a camera.
///
/// The fields have the same meaning as in TangoCameraIntrinsics.
typedef struct Tango3DR_CameraCalibration {
  /// The type of distortion model used. This determines the meaning of the
  /// distortion coefficients.
  Tango3DR_TangoCalibrationType calibration_type;

  /// The width of the image in pixels.
  uint32_t width;
  /// The height of the image in pixels.
  uint32_t height;

  /// Focal length, x axis, in pixels.
  double fx;
  /// Focal length, y axis, in pixels.
  double fy;
  /// Principal point x coordinate on the image, in pixels.
  double cx;
  /// Principal point y coordinate on the image, in pixels.
  double cy;

  /// Distortion coefficients.
  double distortion[5];
} Tango3DR_CameraCalibration;

/// The Tango3DR_Pose struct contains pose information.
///
/// The pose information is commonly copied from TangoPoseData.
typedef struct Tango3DR_Pose {
  /// Translation, ordered x, y, z, of the pose of the target frame with
  /// reference to the base frame.
  double translation[3];

  /// Orientation, as a quaternion, of the pose of the target frame with
  /// reference to the base frame.
  /// Specified as (x,y,z,w) where RotationAngle is in radians:
  /// @code
  ///   x = RotationAxis.x * sin(RotationAngle / 2)
  ///   y = RotationAxis.y * sin(RotationAngle / 2)
  ///   z = RotationAxis.z * sin(RotationAngle / 2)
  ///   w = cos(RotationAngle / 2)
  /// @endcode
  double orientation[4];
} Tango3DR_Pose;

/// The Tango3DR_Matrix3x3 struct contains a single 3x3 matrix.
typedef struct Tango3DR_Matrix3x3 {
  /// Matrix values stored in column-major order (r00, r10, r20, r01,
  /// r11, r21, r02, r12, r22).
  double data[9];
} Tango3DR_Matrix3x3;

/// The Tango3DR_SignedDistanceVoxel struct contains a single voxel.
typedef struct Tango3DR_SignedDistanceVoxel {
  /// Signed distance function, in normalized units.
  int16_t sdf;
  /// Observation weight.
  uint16_t weight;
} Tango3DR_SignedDistanceVoxel;

/// The Tango3DR_GridIndexArray struct contains indices into a
/// three-dimensional grid of cells.  Each grid cell contains voxels
/// arranged in a 16x16x16 cube.
typedef struct Tango3DR_GridIndexArray {
  /// The number of indices in the <code>indices</code> field.
  uint32_t num_indices;

  /// Indices into a three-dimensional grid of cells.
  Tango3DR_GridIndex* indices;
} Tango3DR_GridIndexArray;

/// The Tango3DR_PointCloud struct contains depth information.
///
/// The depth information is commonly copied from TangoXYZij.
typedef struct Tango3DR_PointCloud {
  /// Time of capture of the depth data for this struct (in seconds).
  double timestamp;

  /// The number of points in the <code>points</code> field.
  uint32_t num_points;

  /// Depth points with confidence stored in XYZC order.  XYZ together
  /// refers to a single point. C refers to the confidence in that
  /// point in the range 0.0 - 1.0.
  Tango3DR_Vector4* points;
} Tango3DR_PointCloud;

/// The Tango3DR_ImageBuffer struct contains information about a byte
/// buffer holding image data.
///
/// The image data is commonly copied from TangoImageBuffer.
typedef struct Tango3DR_ImageBuffer {
  /// The width of the image data.
  uint32_t width;
  /// The height of the image data.
  uint32_t height;
  /// The number of bytes per scanline of image data.
  uint32_t stride;
  /// The timestamp of this image.
  double timestamp;
  /// Pixel format of data.
  Tango3DR_ImageFormatType format;
  /// Pixels in the format of this image buffer.
  uint8_t* data;
} Tango3DR_ImageBuffer;

/// A mesh, described by vertices and face indices, with optional
/// per-vertex normals, colors, and textures.
typedef struct Tango3DR_Mesh {
  /// The timestamp of mesh generation (in seconds).
  double timestamp;

  /// Number of valid vertices stored in the <code>vertices</code> array.
  ///
  /// If <code>normals</code>, <code>colors</code>, or
  /// <code>texture_coords</code> is nonnull, this corresponds to the
  /// number of valid elements in those arrays as well.
  uint32_t num_vertices;

  /// Number of valid faces stored in the <code>faces</code> array.
  uint32_t num_faces;

  /// Number of valid textures stored in the <code>textures</code> array.
  uint32_t num_textures;

  /// Maximum capacity of the <code>vertices</code> array.
  ///
  /// If <code>normals</code>, <code>colors</code>, or
  /// <code>texture_coords</code> is nonnull, this corresponds to the
  /// maximum capacity in those arrays as well.
  uint32_t max_num_vertices;

  /// Maximum capacity of the <code>faces</code> array.
  uint32_t max_num_faces;

  /// Maximum capacity of the <code>textures</code>
  uint32_t max_num_textures;  // Capacity of the texture array.

  /// Vertex buffer stored in XYZ order.
  Tango3DR_Vector3* vertices;

  /// Index buffer stored as a triangle list of 32 bit integers.
  Tango3DR_Face* faces;

  /// Normal buffer stored in XYZ order.
  Tango3DR_Vector3* normals;

  /// Color buffer stored in RGBA order.
  Tango3DR_Color* colors;

  /// Texture coordinate buffer stored in UV order.
  Tango3DR_TexCoord* texture_coords;

  /// Mapping of faces to texture images. -1 means no texture assigned.
  int32_t* texture_ids;

  /// Array of texture images.
  Tango3DR_ImageBuffer* textures;
} Tango3DR_Mesh;

/// Struct representing a single building level.
typedef struct Tango3DR_FloorplanLevel {
  /// Vertical position of the floor (in meters).
  float min_z;

  /// Vertical position of the ceiling (in meters).
  float max_z;
} Tango3DR_FloorplanLevel;

/// Struct representing building levels.
typedef struct Tango3DR_FloorplanLevelArray {
  /// Number of building levels.
  uint32_t num_levels;

  /// Array with building level information.
  Tango3DR_FloorplanLevel* levels;
} Tango3DR_FloorplanLevelArray;

/// An array of two floats, commonly a 2D position.
typedef float Tango3DR_Vector2[2];

/// Enumeration of floor plan layers.
typedef enum {
  TANGO_3DR_LAYER_SPACE = 0,
  TANGO_3DR_LAYER_WALLS,
  TANGO_3DR_LAYER_FURNITURE,
} Tango3DR_FloorplanLayer;

/// Struct representing a single 2D polyline or polygon.
typedef struct Tango3DR_Polygon {
  /// Number of vertices contained in the <code>vertices</code> array.
  uint32_t num_vertices;

  /// If true, indicates that the path is closed, i.e., the last and the first
  /// vertex are connected.
  bool closed;

  /// Floor plan layer to which this path belongs to.
  Tango3DR_FloorplanLayer layer;

  /// 2D points.
  Tango3DR_Vector2* vertices;  // In meters.

  /// Surface area of the path. A negative number indicates that this polygon
  /// represents a hole (in a bigger polygon).
  double area;  // In square meters.
} Tango3DR_Polygon;

/// 2D vector graphics object.
/// The polygons are sorted by decreasing surface area, so that it is safe to
/// render them directly using the provided ordering. Note that polygons with
/// negative surface area indicate holes that need to be rendered in the
/// background color.
typedef struct Tango3DR_PolygonArray {
  /// Number of paths contained in the <code>paths</code> array.
  uint32_t num_polygons;

  /// Array containing 2D polylines/polygons.
  Tango3DR_Polygon* polygons;
} Tango3DR_PolygonArray;

/// Create a point cloud.
///
/// @param num_points How many points the newly allocated point cloud
///     should be able to hold.
/// @return A freshly allocated Tango3DR_PointCloud.
Tango3DR_PointCloud* Tango3DR_PointCloud_create(const uint32_t num_points);

/// Initialize an empty point cloud.
///
/// @param cloud Pointer to point cloud.
/// @return @c TANGO_3DR_SUCCESS on successfully initializing the struct.
///     Returns @c TANGO_3DR_INVALID if mesh is NULL.
Tango3DR_Status Tango3DR_PointCloud_createEmpty(Tango3DR_PointCloud* cloud);

/// Destroy a previously created point cloud.
///
/// @param cloud Pointer to a previously created cloud.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the point cloud.  Returns @c TANGO_3DR_INVALID if cloud is
///     NULL.
Tango3DR_Status Tango3DR_PointCloud_destroy(Tango3DR_PointCloud* cloud);

/// Initialize an empty calibration struct.
///
/// @param calibration Pointer to camera calibration.
/// @return @c TANGO_3DR_SUCCESS on successfully initializing the struct.
///     Returns @c TANGO_3DR_INVALID if calibration is NULL.
Tango3DR_Status Tango3DR_CameraCalibration_createEmpty(
    Tango3DR_CameraCalibration* calibration);

/// Initialize an empty image buffer.
///
/// @param image Pointer to image buffer.
/// @return @c TANGO_3DR_SUCCESS on successfully initializing the buffer.
///     Returns @c TANGO_3DR_INVALID if image is NULL.
Tango3DR_Status Tango3DR_ImageBuffer_createEmpty(Tango3DR_ImageBuffer* image);

/// Create a mesh.
///
/// @param vertices_capacity Maximum number of vertices this mesh can
///     hold.
/// @param faces_capacity Maximum number of faces this mesh can hold.
/// @param allocate_normals If the normal buffer should be allocated.
/// @param allocate_colors If the color buffer should be allocated.
/// @param allocate_tex_coords If the texture coordinate buffer should
///     be allocated.
/// @param textures_capacity Maximum number of textures this mesh can
///     hold.
/// @param textures_width Width in pixels of the texture image buffers
///     allocated.
/// @param textures_height Height in pixels of the texture image
///     buffers allocated.
/// @return A freshly allocated Tango3DR_Mesh.
Tango3DR_Mesh* Tango3DR_Mesh_create(
    const uint32_t vertices_capacity, const uint32_t faces_capacity,
    const bool allocate_normals, const bool allocate_colors,
    const bool allocate_tex_coords, const bool allocate_tex_ids,
    const uint32_t textures_capacity, const uint32_t textures_width,
    const uint32_t textures_height);

/// Load a mesh from a .obj file.
///
/// @param path Path to the .obj mesh.
/// @param mesh On successful return, this will be filled
///     with a pointer to a freshly allocated Tango3DR_Mesh. After
///     use, free this by calling Tango3DR_Mesh_destroy().
/// @return @c TANGO_3DR_SUCCESS on successfully loading a mesh, 3D
///     Reconstruction, @c TANGO_3DR_INVALID if the parameters are not
///     valid, TANGO_3DR_ERROR if using loading failed.
Tango3DR_Status Tango3DR_Mesh_loadFromObj(const char* const path,
                                          Tango3DR_Mesh** mesh);

/// Save a mesh to an .obj file.
///
/// @param mesh Mesh to be written.
/// @param path On successful return, this file will contain the mesh in OBJ
///     format.
/// @return @c TANGO_3DR_SUCCESS on successfully saving a mesh, 3D
///     Reconstruction, @c TANGO_3DR_INVALID if the parameters are not
///     valid, TANGO_3DR_ERROR if using saving failed.
Tango3DR_Status Tango3DR_Mesh_saveToObj(const Tango3DR_Mesh* mesh,
                                        const char* const path);

/// Initialize an empty mesh struct.
///
/// @param mesh Pre-allocated mesh struct.
/// @return @c TANGO_3DR_SUCCESS on successfully initializing the mesh.
///     Returns @c TANGO_3DR_INVALID if mesh is NULL.
Tango3DR_Status Tango3DR_Mesh_createEmpty(Tango3DR_Mesh* mesh);

/// Destroy a previously created mesh.
///
/// @param mesh Pointer to a previously created mesh.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the mesh.  Returns @c TANGO_3DR_INVALID if cloud is NULL.
Tango3DR_Status Tango3DR_Mesh_destroy(Tango3DR_Mesh* mesh);

/// Create an image buffer.
///
/// @param width
/// @param height
/// @param format
/// @return A freshly allocated Tango3DR_ImageBuffer.
Tango3DR_ImageBuffer* Tango3DR_ImageBuffer_create(
    uint32_t width, uint32_t height, Tango3DR_ImageFormatType format);

/// Destroy a previously created image buffer.
///
/// @param image Pointer to a previously created image buffer.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the image buffer.  Returns @c TANGO_3DR_INVALID if image is
///     NULL.
Tango3DR_Status Tango3DR_ImageBuffer_destroy(Tango3DR_ImageBuffer* image);

/// Create a grid index array.
///
/// @param num_indices How many grid indecies the newly allocated grid
///     index array should be able to hold.
/// @return A freshly allocated Tango3DR_GridIndexArray.
Tango3DR_GridIndexArray* Tango3DR_GridIndexArray_create(
    const uint32_t num_indices);

/// Destroy a previously created grid index array.
///
/// @param grid_index_array Pointer to a previously created grid index
///     array.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the grid index array.  Returns @c TANGO_3DR_INVALID if
///     grid_index_array is NULL.
Tango3DR_Status Tango3DR_GridIndexArray_destroy(
    Tango3DR_GridIndexArray* grid_index_array);

/**@} */

/// @defgroup ConfigParams Configuration Parameters Get and Set Functions
/// @brief Configuration Parameters Get and Set Functions
///
/// For an allocated Tango3DR_ConfigH, these functions get and set
/// parameters of that config.  The parameters that are available are
/// dependent on the type of configuration created.
///
/// You can use the handle to query the current state, or you can
/// create a new handle and alter its contents to set settings. For
/// each type of configuration parameter (bool, double, int32, etc)
/// you call the corresponding get/set function, such as
/// TangoConfig_getBool() for a boolean.
///
/// The supported configuration parameters that can be set for
/// TANGO_3DR_CONFIG_CONTEXT are:
///
/// <table>
/// <tr><td>boolean use_parallel_integration</td><td>
///         When true, will use multi-threading for the reconstruction
///         updates.  Default is false.</td></tr>
///
/// <tr><td>double resolution</td><td>
///         The reconstruction resolution in meters.  Default is
///         0.03</td></tr>
///
/// <tr><td>boolean generate_color</td><td>
///         Whether the meshes should contain vertex colors.  Default
///         is true.</td></tr>
///
/// <tr><td>double min_depth</td><td>
///         Minimum depth range for input to the reconstruction in
///         meters.  Default is 0.60.</td></tr>
///
/// <tr><td>double max_depth</td><td>
///         Maximum depth range for input to the reconstruction in
///         meters.  Default is 3.50.</td></tr>
///
/// <tr><td>boolean use_space_clearing</td><td>
///         When true, will clear away free space along the depth
///         observations. Default is false.  If enabled, 3D
///         reconstruction will create higher quality meshes at a cost
///         to performance.</td></tr>
///
/// <tr><td>matrix3x3 external_T_tango</td><td>
///         Transformation matrix that is left-multiplied to any point
///         generated.  Defaults to the identity matrix.  This is
///         useful to convert meshes generated to a left-handed
///         coordinate system or to some other coordinate system
///         convention than Tango's.</td></tr>
///
/// <tr><td>boolean use_clockwise_winding_order</td><td>
///         When true, faces will have a clockwise (CW) winding order
///         (in contrast to counter-clockwise (CCW) winding order,
///         which is the default).  Note that this is done before
///         applying external_T_tango, so it has the opposite result
///         if external_T_tango switches handedness.</td></tr>
///
/// <tr><td>int32 max_voxel_weight</td><td>
///         The maximum voxel weight for the integration
///         filter. Defaults to 16383.  Roughly corresponds to the
///         number of observations. A higher value gives better
///         geometric quality and stability; a lower value gives more
///         responsiveness to dynamic objects.</td></tr>
///
/// <tr><td>int32 min_num_vertices</td><td>
///         Minimum number of vertices that every connected component
///         in the mesh should consist of. Defaults to 1.  Smaller
///         components (typically noise) will be deleted.</td></tr>
///
/// <tr><td>int64 dataset_start_time</td><td>
///         Start time in ns. Defaults to 0. When reconstructing or
///         texturing from a dataset, all messages with timestamps <
///         dataset_start_time will be ignored.</td></tr>
///
/// <tr><td>int32_t update_method</td><td>
///         Update method to be used when caling @c Tango3DR_update. See
///         @c Tango3DR_UpdateMethod. </td></tr>
///
/// <tr><td>boolean use_floorplan</td><td>
///         When true, enable floor plan mode. Note that this switches the mesh
///         to an stylized/extruded mesh of the floor plan. Default is false.
///
/// <tr><td>boolean use_floorplan_canonical_orientation</td><td>
///         Rotate the floor plan so that most dominant direction is aligned
///         with the x axis. Default is false.
///
/// <tr><td>double floorplan_max_error</td><td>
///         Upper bound on geometric error in meters during polygon
///         simplification. Set to zero to disable simplification.
/// </td></tr>
///
/// </table>
///
/// The supported configuration parameters that can be set for
/// TANGO_3DR_CONFIG_TEXTURING are:
///
/// <table>
///
/// <tr><td>int32_t texturing_backend</td><td>
///         Backend to use when performing mesh texturing. See
///         @c Tango3DR_TexturingBackend. </td></tr>
///
/// <tr><td>int32 mesh_simplification_factor</td><td>
///         Reduce the number of vertices by this factor before
///         generating the texture.  Defaults to 30.</td></tr>
///
/// <tr><td>int32 texture_size</td><td>
///         The texture size in pixels.  Defaults to 2048.</td></tr>
///
/// <tr><td>double bevel</td><td>
///         The bevel size in pixels.  Defaults to 3.0.</td></tr>
///
/// <tr><td>double min_resolution</td><td>
///         The minimum resolution in meters/pixels.  Defaults to 0.0.
///         This parameter may be overridden by max_num_textures if in
///         conflict.</td></tr>
///
/// <tr><td>int32 max_num_textures</td><td>
///         The maximum number of texture images to generate.  Defaults to 0
///         (unlimited).  This parameter overrides min_resolution if in
///         conflict. </td></tr>
///
/// <tr><td>int32 downsample</td><td>
///         Use only every n-th color image.  Defaults to 1.</td></tr>
///
/// <tr><td>matrix4x4 external_T_tango</td><td>
///         Transformation matrix that is left-multiplied to any point
///         generated.  Defaults to the identity matrix.  This is
///         useful to convert meshes generated to a left-handed
///         coordinate system or to some other coordinate system
///         convention than Tango's.</td></tr>
///
/// @{


/// Creates a Tango3DR_ConfigH object with configuration settings.
/// This should be used to initialize a Config object for setting the
/// configuration that Tango 3D Reconstruction should be run in. The
/// Config handle is passed to Tango3DR_create() or
/// Tango3DR_textureMeshFromDataset().  The handle is needed only at
/// the time of the call where it is used and can safely be freed
/// after it has been used in Tango3DR_create() or
/// Tango3DR_textureMeshFromDataset().
///
/// @param config_type The requested configuration type.
/// @return A handle (Tango3DR_ConfigH) for a newly allocated
///     config object with settings as requested by
///     config_type. Returns NULL if the config_type is not valid, the
///     config could not be allocated, or an internal failure
///     occurred.
Tango3DR_ConfigH Tango3DR_Config_create(Tango3DR_ConfigType config_type);

/// Free a Tango3DR_ConfigH object.
///
/// Frees the Tango3DR_ConfigH object for the handle specified by the
/// config variable.
Tango3DR_Status Tango3DR_Config_destroy(Tango3DR_ConfigH config);

/// Set a boolean configuration parameter.
/// @param config The configuration object to set the parameter on. @p config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p config or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_Config_setBool(Tango3DR_ConfigH config,
                                        const char* key, bool value);

/// Set an int32 configuration parameter.
/// @param config The configuration object to set the parameter on. @p config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p config or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_Config_setInt32(Tango3DR_ConfigH config,
                                         const char* key, int32_t value);

/// Set an int64 configuration parameter.
/// @param config The configuration object to set the parameter on. @p config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p config or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_Config_setInt64(Tango3DR_ConfigH config,
                                         const char* key, int64_t value);

/// Set a double configuration parameter.
/// @param config The configuration object to set the parameter on. @p config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p config or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_Config_setDouble(Tango3DR_ConfigH config,
                                          const char* key, double value);

/// Set a matrix3x3 configuration parameter.
/// @param config The configuration object to set the parameter on. @p config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p config or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_Config_setMatrix3x3(Tango3DR_ConfigH config,
                                             const char* key,
                                             const Tango3DR_Matrix3x3* value);

/// Get a boolean configuration parameter.
/// @param config The configuration object to get the parameter from. config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_Config_getBool(const Tango3DR_ConfigH config,
                                        const char* key, bool* value);

/// Get an int32 configuration parameter.
/// @param config The configuration object to get the parameter from. config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_Config_getInt32(const Tango3DR_ConfigH config,
                                         const char* key, int32_t* value);

/// Get an int64 configuration parameter.
/// @param config The configuration object to get the parameter from. config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_Config_getInt64(const Tango3DR_ConfigH config,
                                         const char* key, int64_t* value);

/// Get a double configuration parameter.
/// @param config The configuration object to get the parameter from. config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_Config_getDouble(const Tango3DR_ConfigH config,
                                          const char* key, double* value);

/// Get a matrix3x3 configuration parameter.
/// @param config The configuration object to get the parameter from. config
///     must have been created with Tango3DR_Config_create().
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_Config_getMatrix3x3(Tango3DR_ConfigH config,
                                             const char* key,
                                             Tango3DR_Matrix3x3* value);

/**@} */


/// @defgroup MeshBuildingFunctions Mesh Building Functions
/// @brief Core API for building 3D meshes.
///
/// To build a mesh on a Tango device in realtime with the API, follow
/// the following steps:
///
/// - Create a Reconstruction server with Tango3DR_create().
///
/// - In the Tango Service callbacks, call Tango3DR_update() with
///   PointCloud and ImageBuffers for XYZij and Color camera
///   callbacks.  This will update internal state and fill out a
///   Tango3DR_GridIndexArray.
///
/// - For each GridIndex in the Tango3DR_GridIndexArray filled out
///   above, call Tango3DR_extractMeshSegment or
///   Tango3DR_extractPreallocatedMeshSegment.  This is commonly done
///   in your app's main thread or GL thread.
///
/// - (Optional) After creating the entire mesh, you can get a more
///   optimized mesh by extracting it all in one pass.  Call one of
///   Tango3DR_extractMesh, Tango3DR_extractPreallocatedMesh,
///   Tango3DR_extractFullMesh, or
///   Tango3DR_extractPreallocatedFullMesh.
///
/// - Finally, when completely finished, call Tango3DR_destroy to
///   clean up the reconstruction server.
///
/// During the lifetime of a reconstruction server, the following runtime
/// parameters can be changed using Tango3DR_setRuntimeBool:
///
/// <table>
/// <tr><td>boolean use_floorplan</td><td>
///         When true, extract a 2D floor plan instead of a full 3D mesh.
/// </td></tr>
/// </table>
///
///
/// @{

/// Create a new Tango 3D Reconstruction server and return a handle to
/// it.  You can run as many 3D Reconstruction servers as you want.
///
/// @param context_config The server will be started with the setting
///     specified by this config handle.  If NULL is passed here, then
///     the server will be started in the default configuration.
///
/// @return A handle to the Tango 3D Reconstruction server.
Tango3DR_Context Tango3DR_create(const Tango3DR_ConfigH context_config);

/// Destroy a previously created Tango 3D Reconstruction server.
///
/// @param context Handle to a previously created server.
///
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the server.  Returns @c TANGO_3DR_INVALID if context is NULL.
Tango3DR_Status Tango3DR_destroy(Tango3DR_Context context);

/// Create a new Tango 3D texturing context and return a handle to it.
///
/// @param context_config The server will be started with the setting
///     specified by this config handle.  If NULL is passed here, then
///     the server will be started in the default configuration.
/// @param path to the Tango dataset containing the calibration file.
/// @param tango_mesh mesh that will be textured.
///
/// @return A handle to the Tango texturing context.
Tango3DR_TexturingContext Tango3DR_createTexturingContext(
    const Tango3DR_ConfigH texture_config, const char* const dataset_path,
    const Tango3DR_Mesh* tango_mesh);

/// Destroy a previously created texturing context.
///
/// @param context Handle to a previously created context.
///
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the context.  Returns @c TANGO_3DR_INVALID if context is NULL.
Tango3DR_Status Tango3DR_destroyTexturingContext(
    Tango3DR_TexturingContext context);

/// Set a boolean runtime parameter.
/// @param context Handle to a previously created server.
/// @param key The string key value of the configuration parameter to set.
/// @param value The value to set the configuration key to.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if @p context or key is NULL, or key is not
///     found or could not be set.
Tango3DR_Status Tango3DR_setRuntimeBool(Tango3DR_Context context,
                                        const char* key, bool value);

/// Get a boolean runtime parameter.
/// @param context Handle to a previously created server.
/// @param key The string key value of the configuration parameter to get.
/// @param value Upon success, set to the value for the configuration key.
/// @return @c TANGO_3DR_SUCCESS on success or @c
///     TANGO_3DR_INVALID if the any of the arguments is NULL, or if
///     the key could not be found.
Tango3DR_Status Tango3DR_getRuntimeBool(Tango3DR_Context context,
                                        const char* key, bool* value);

/// Clear all voxels from a running server.
///
/// @param context Handle to a previously created server.
///
/// @return @c TANGO_3DR_SUCCESS on successfully clearing the
///     server's voxels.  Returns @c TANGO_3DR_INVALID if context is
///     NULL.
Tango3DR_Status Tango3DR_clear(Tango3DR_Context context);

/// Updates the voxels using a point cloud and image.
///
/// A list of affected voxel cells are returned so that you can create
/// new meshes for only those cells via Tango3DR_extractMeshSegment(),
/// Tango3DR_extractPreallocatedMeshSegment(),
/// Tango3DR_updateFloorplanSegment() or Tango3DR_extractFloorplanSegment().
///
/// @param context Handle to a previously created server.
/// @param cloud Point cloud depth data.
/// @param cloud_pose Pose of the Tango device when capturing the data
///     in @c cloud.
/// @param image Image data.  If NULL, no color information will be
///     generated for affected voxels.
/// @param image_pose Pose of the Tango device when capturing the data
///     in @c image.  Can be NULL if @c image is NULL.
/// @param calibration Camera calibration for the captured image.  Can
///     be NULL if @c image is NULL.
/// @param updated_indices On successful return, this will be filled
///     with a pointer to a freshly allocated Tango3DR_GridIndexArray
///     describing the grid cells whose voxels are affected.  After
///     use, free this by calling Tango3DR_GridIndexArray_destroy().
/// @return @c TANGO_3DR_SUCCESS on successfully updating 3D
///     Reconstruction, @c TANGO_3DR_INVALID if the parameters are not
///     valid, TANGO_3DR_ERROR if using @c TANGO_3DR_PROJECTIVE_UPDATE but
///     the @c and @c Tango3DR_setDepthCalibration method hasn't been called.
Tango3DR_Status Tango3DR_update(Tango3DR_Context context,
                                const Tango3DR_PointCloud* cloud,
                                const Tango3DR_Pose* cloud_pose,
                                const Tango3DR_ImageBuffer* image,
                                const Tango3DR_Pose* image_pose,
                                Tango3DR_GridIndexArray** updated_indices);

/// Retrieves the indices of all active cells.
///
/// A cell is marked as active if it has any voxels that have been
/// updated by Tango3DR_update() and will remain active until
/// reconstruction is cleared using Tango3DR_clear().
///
/// @param context Handle to a previously created server.
/// @param updated_indices On successful return, this will be filled
///     with a pointer to a freshly allocated Tango3DR_GridIndexArray
///     containing all grid cells that are active.  After
///     use, free this by calling Tango3DR_GridIndexArray_destroy().
/// @return @c TANGO_3DR_SUCCESS on success and @c
///     TANGO_3DR_INVALID if the parameters are not valid.
Tango3DR_Status Tango3DR_getActiveIndices(
    const Tango3DR_Context context, Tango3DR_GridIndexArray** active_indices);

/// Extract a mesh for the current state of a grid cell.
///
/// @param context Handle to a previously created server.
/// @param grid_index Index for the grid cell to extract.
/// @param mesh On successful return, a freshly allocated
///     Tango3DR_Mesh containing the mesh for the grid cell.  After
///     use, free this by calling Tango3DR_Mesh_destroy().
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractMeshSegment(const Tango3DR_Context context,
                                            const Tango3DR_GridIndex grid_index,
                                            Tango3DR_Mesh** mesh);

/// Extract a mesh for the current state of a grid cell.
///
/// @param context Handle to a previously created server.
/// @param grid_index Index for the grid cell to extract.
/// @param mesh Pointer to a previously allocated mesh. On successful
///     return the buffers of the mesh will be updated to represent
///     the mesh.  When @c TANGO_3DR_INSUFFICIENT_SPACE is returned,
///     as much of the mesh as possible will be filled in.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INSUFFICIENT_SPACE if extraction was successful but
///     there was not enough space for all the buffers, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractPreallocatedMeshSegment(
    const Tango3DR_Context context, const Tango3DR_GridIndex grid_index,
    Tango3DR_Mesh* mesh);

/// Extract a single mesh for the current state of multiple grid
/// cells.
///
/// @param context Handle to a previously created server.
/// @param grid_index_array Array of grid cells to extract.
/// @param mesh On successful return, a freshly allocated
///     Tango3DR_Mesh containing the mesh for the grid cells.  After
///     use, free this by calling Tango3DR_Mesh_destroy().
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractMesh(
    const Tango3DR_Context context,
    const Tango3DR_GridIndexArray* grid_index_array, Tango3DR_Mesh** mesh);

/// Extract a single mesh for the current state of multiple grid
/// cells.
///
/// @param context Handle to a previously created server.
/// @param grid_index_array Array of grid cells to extract.
/// @param mesh Pointer to a previously allocated mesh. On successful
///     return the buffers of the mesh will be updated to represent
///     the mesh.  When @c TANGO_3DR_INSUFFICIENT_SPACE is returned,
///     as much of the mesh as possible will be filled in.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INSUFFICIENT_SPACE if extraction was successful but
///     there was not enough space for all the buffers, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractPreallocatedMesh(
    const Tango3DR_Context context,
    const Tango3DR_GridIndexArray* grid_index_array, Tango3DR_Mesh* mesh);

/// Extract a single mesh for the current state of all active grid
/// cells.
///
/// @param context Handle to a previously created server.
/// @param mesh On successful return, a freshly allocated
///     Tango3DR_Mesh containing the mesh for all active grid cells.
///     After use, free this by calling Tango3DR_Mesh_destroy().
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractFullMesh(const Tango3DR_Context context,
                                         Tango3DR_Mesh** mesh);

/// Extract a single mesh for the current state of all active grid
/// cells.
///
/// @param context Handle to a previously created server.
/// @param mesh Pointer to a previously allocated mesh. On successful
///     return the buffers of the mesh will be updated to represent
///     the mesh.  When @c TANGO_3DR_INSUFFICIENT_SPACE is returned,
///     as much of the mesh as possible will be filled in.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INSUFFICIENT_SPACE if extraction was successful but
///     there was not enough space for all the buffers, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractPreallocatedFullMesh(
    const Tango3DR_Context context, Tango3DR_Mesh* mesh);

/// Extract voxels for the current state of a grid cell.
///
/// @param context Handle to a previously created server.
/// @param grid_index Index for the grid cell to extract.
/// @param num_sdf_voxels Number of voxels in this grid cell.  Must be
///     16x16x16 = 4096.
/// @param sdf_voxels Pointer to a previously allocated list of
///     voxels. On successful return sdf_voxels will be updated to
///     represent the current state.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_extractPreallocatedVoxelGridSegment(
    const Tango3DR_Context context, const Tango3DR_GridIndex grid_index,
    const int num_sdf_voxels, Tango3DR_SignedDistanceVoxel* sdf_voxels);

/// Sets the depth camera intrinsic calibration. It is required to call this
/// function once before calling @c Tango3DR_update if the update_method flag is
/// set to @c TANGO_3DR_PROJECTIVE_UPDATE.
///
/// @param context Handle to a previously created server.
/// @param calibration The calibration parameters of the depth camera.
/// @return @c TANGO_3DR_SUCCESS on success, @TANGO_3DR_INVALID if the
///     parameters are not valid.
Tango3DR_Status Tango3DR_setDepthCalibration(
    const Tango3DR_Context context,
    const Tango3DR_CameraCalibration* calibration);

/// Sets the depth camera intrinsic calibration. It is required to call this
/// function once before calling @c Tango3DR_update if you are passing color
/// images to the reconstruction.
///
/// @param context Handle to a previously created server.
/// @param calibration The calibration parameters of the color camera.
/// @return @c TANGO_3DR_SUCCESS on success, @TANGO_3DR_INVALID if the
///     parameters are not valid.
Tango3DR_Status Tango3DR_setColorCalibration(
    const Tango3DR_Context context,
    const Tango3DR_CameraCalibration* calibration);

/**@} */

/// @cond
/// @defgroup Datasets Functions that require a dataset.
/// @brief Functions that require a dataset.
/// @{

/// A callback function for dataset processing to report progress.
typedef void (*Tango3DR_ProgressCallback)(int progress, void* callback_param);

/// Reads a dataset with tracking images and IMU data from storage and
/// generate an optimized trajectory using bundle adjustment.
///
/// @param dataset_path Path to a dataset file.
/// @param loop_closure_database_path Path to a the loop closure database file,
///     distributed with the Tango 3D reconstruction library. Can be NULL, in
///     which case the default on-device location will be used.
/// @param trajectory On successful return, a freshly allocated
///     trajectory representing the path a Tango device traveled over
///     time.
/// @param progress_callback Called periodically while creating the
///     trajectory.  Optional.
/// @param callback_param Value passed as @c callback_param to the
///     callback.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_createTrajectoryFromDataset(
    const char* dataset_path, const char* loop_closure_database_path,
    Tango3DR_TrajectoryH* trajectory,
    Tango3DR_ProgressCallback progress_callback, void* callback_param);

/// Update the voxels given a trajectory and dataset.
///
/// @param context Handle to a previously created server.
/// @param dataset_path Path to a dataset file.
/// @param trajectory Handle to a trajectory representing the path a
///     Tango device traveled over time.
/// @param progress_callback Called periodically while creating the
///     trajectory.  Optional.
/// @param callback_param Value passed as @c callback_param to the
///     callback.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_updateFromTrajectoryAndDataset(
    const Tango3DR_Context context, const char* dataset_path,
    const Tango3DR_TrajectoryH trajectory,
    Tango3DR_ProgressCallback progress_callback, void* callback_param);

/// Extract a mesh with textures given a trajectory and dataset.
///
/// @param texture_config Configuration for texturing.  If NULL is
///     passed here, texturing will be done with the default
///     configuration.
/// @param dataset_path Path to a dataset file.
/// @param trajectory Handle to a trajectory representing the path a
///     Tango device traveled over time.
/// @param tango_mesh_in Mesh to texture.  Commonly previously
///     extracted from Tango3DR_extractFullMesh().
/// @param tango_mesh_out On successful return, a freshly allocated
///     Tango3DR_Mesh containing the mesh with texture processing.
///     After use, free this by calling Tango3DR_Mesh_destroy().
/// @param progress_callback Called periodically while texturing the
///     mesh.  Optional.
/// @param callback_param Value passed as @c callback_param to the
///     callback.
/// @return @c TANGO_3DR_SUCCESS on success, @c
///     TANGO_3DR_INVALID if the parameters are not valid, and @c
///     TANGO_3DR_ERROR if some other error occurred.
Tango3DR_Status Tango3DR_textureMeshFromDataset(
    const Tango3DR_ConfigH texture_config, const char* dataset_path,
    const Tango3DR_TrajectoryH trajectory, const Tango3DR_Mesh* tango_mesh_in,
    Tango3DR_Mesh** tango_mesh_out, Tango3DR_ProgressCallback progress_callback,
    void* callback_param);

// TODO(idryanov): Add link to documentation.
/// Extract the raw sensor data (images, point clouds, IMU) from a Tango dataset
/// and saves it in human-readable form. Running this function might take some
/// time.
///
/// @param dataset_path Path to a dataset.
/// @param output_path Path to the output folder.
/// @param progress_callback Called periodically while exporting the dataset.
///     Optional, can be set to NULL.
/// @param callback_param Value passed as @c callback_param to the
///     callback.
Tango3DR_Status TangoDataset_extractRawDataFromDataset(
    const char* dataset_path, const char* output_path,
    Tango3DR_ProgressCallback progress_callback, void* callback_param);

// TODO(idryanov): Add unit tests for trajectories (b/33011366).
/// Destroy a previously created trajectory.
///
/// @param trajectory Handle to a previously created trajectory.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
///     the trajectory.  Returns @c TANGO_3DR_INVALID if trajectory is
///     NULL.
Tango3DR_Status Tango3DR_Trajectory_destroy(Tango3DR_TrajectoryH trajectory);

/// Retrieves the pose at a given time from a trajectory.
///
/// @param trajectory The handle to a Tango trajectory.
/// @param timestamp The timestamp, in seconds, at which to query the pose.
/// @param tango_pose A pointer to the output pose. Must be within the start
///     and end time of the trajectory.
/// @return @c TANGO_3DR_SUCCESS on success, @c TANGO_3DR_INVALID if the
///     parameters are not valid, and @c TANGO_3DR_ERROR if the timestamp
///     queried is not in the range of the trajectory.
Tango3DR_Status Tango3DR_getPoseAtTime(const Tango3DR_TrajectoryH trajectory,
                                       const double timestamp,
                                       Tango3DR_Pose* tango_pose);

/// Updates the texturing context using an image with a corresponding pose. Must
/// be called from the same thread where the context was created and destroyed.
///
/// @param context Handle to the texturing context.
/// @param image Image used for texturing.
/// @param image_pose Pose corresponding to the image.
/// @return @c TANGO_3DR_SUCCESS on success and @c TANGO_3DR_INVALID if the
///     parameters are not valid.
Tango3DR_Status Tango3DR_updateTexture(Tango3DR_TexturingContext context,
                                       const Tango3DR_ImageBuffer* image,
                                       const Tango3DR_Pose* image_pose);

/// Extracts the textured mesh from the texturing context.
///
/// @param context Handle to the texturing context.
/// @param mesh On successful return, a freshly allocated Tango3DR_Mesh
///     containing the mesh for the grid cell.  After use, free this by calling
///     Tango3DR_Mesh_destroy().
/// @return @c TANGO_3DR_SUCCESS on success, @c TANGO_3DR_INVALID if the
///     parameters are not valid
Tango3DR_Status Tango3DR_getTexturedMesh(
    const Tango3DR_TexturingContext context, Tango3DR_Mesh** tango_mesh_out);

/**@} */
/// @endcond

/// Detect and extract (vertical) building levels. Currently, we only support
/// single story buildings, so levels->num_levels will always equal 1.
///
/// @param context Handle to a previously created server.
/// @param graphics On successful return, this will be filled with a pointer
/// to a freshly allocated Tango3DR_Levels object containing the levels
/// of the building.  After use, free this by calling
/// Tango3DR_destroyGraphics().
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
/// the vector graphics. Returns @c TANGO_3DR_INVALID if
/// <code>context</code> or <code>graphics</code> is NULL.
Tango3DR_Status Tango3DR_extractLevels(const Tango3DR_Context context,
                                       Tango3DR_FloorplanLevelArray** levels);

/// Reset and enable automatic building level estimator. To disable the
/// automatic estimator and to specify the building level manually instead,
/// call <code>Tango3DR_selectLevel</code>.
///
/// @param context Handle to a previously created server.
/// @return @c TANGO_3DR_SUCCESS on successfully resetting the building level
/// estimator. Returns @c TANGO_3DR_INVALID if <code>context</code> is NULL.
Tango3DR_Status Tango3DR_resetLevelsEstimator(const Tango3DR_Context context);

/// Manually select building level. This disables the automatic building level
/// estimator. To re-enable the automatic estimator, call <code>
/// Tango3DR_resetLevelsEstimator</code>.
///
/// @param context Handle to a previously created server.
/// @param min_z Floor height, in meters.
/// @param max_z Ceiling height, in meters.
/// @return @c TANGO_3DR_SUCCESS on successfully destroying
/// the vector graphics. Returns @c TANGO_3DR_INVALID if
/// <code>context</code> is NULL.
Tango3DR_Status Tango3DR_selectLevel(const Tango3DR_Context context,
                                     const Tango3DR_FloorplanLevel& level);

/// Destroy a previously created Tango3DR_Levels object.
///
/// @param config Pointer to a previously created vector graphics object.
void Tango3DR_destroyLevels(Tango3DR_FloorplanLevelArray* levels);

/// Update a floor plan in an existing 3D reconstruction context.
/// You need to call this function or Tango3DR_updateFloorplan() prior to
/// extracting a floor plan using Tango3DR_extractFullFloorplan() or
/// Tango3DR_extractFloorplanSegment().
///
/// @param context Handle to a previously created server.
/// @return @c TANGO_3DR_SUCCESS on successfully updating the floor plan.
/// Returns @c TANGO_3DR_INVALID if <code>context</code> is NULL.
Tango3DR_Status Tango3DR_updateFullFloorplan(const Tango3DR_Context context);

/// Update a floor plan in an existing 3D reconstruction context.
/// You need to call this function or Tango3DR_updateFullFloorplan() prior to
/// extracting a floor plan using Tango3DR_extractFullFloorplan() or
/// Tango3DR_extractFloorplanSegment().
///
/// @param context Handle to a previously created server.
/// @param grid_index_array Grid cells that need to be updated.
/// @return @c TANGO_3DR_SUCCESS on successfully updating the floor plan.
/// Returns @c TANGO_3DR_INVALID if <code>context</code> is NULL.
Tango3DR_Status Tango3DR_updateFloorplan(
    const Tango3DR_Context context,
    const Tango3DR_GridIndexArray* grid_index_array);

/// Extract a floor plan from an existing 3D reconstruction context.
///
/// @param context Handle to a previously created server.
/// @param levels Building levels to extract.
/// @param graphics On successful return, this will be filled with a pointer
/// to a freshly allocated Tango3DR_Graphics object containing a vector graphics
/// object. The polygons are sorted by decreasing surface area, so that it is
/// safe to render them directly using the provided ordering. Note that polygons
/// with negative surface area indicate holes that need to be rendered in the
/// background color. After use, free this by calling
/// Tango3DR_destroyGraphics().
/// @return @c TANGO_3DR_SUCCESS on successfully extracting the floor plan.
/// Returns @c TANGO_3DR_INVALID if <code>context</code> or
/// <code>graphics</code> is NULL.
Tango3DR_Status Tango3DR_extractFullFloorplan(const Tango3DR_Context context,
                                              Tango3DR_PolygonArray** graphics);

/// Extract a floor plan segment from an existing 3D reconstruction context.
/// Note that this function takes only a 2D grid index (x,y) instead of a
/// 3D grid index (x,y,z).
///
/// @param context Handle to a previously created server.
/// @param grid_index Index for the grid cell to extract.
/// @param graphics On successful return, this will be filled with a pointer
/// to a freshly allocated Tango3DR_Graphics object containing a vector graphics
/// object. After use, free this by calling Tango3DR_destroyGraphics().
/// @return @c TANGO_3DR_SUCCESS on successfully extracting the floor plan.
/// Returns @c TANGO_3DR_INVALID if <code>context</code> or
/// <code>graphics</code> is NULL.
Tango3DR_Status Tango3DR_extractFloorplanSegment(
    const Tango3DR_Context context, const Tango3DR_GridIndex2D grid_index,
    Tango3DR_PolygonArray** graphics);

/// Destroy a previously created vector graphics object.
///
/// @param config Pointer to a previously created vector graphics object.
void Tango3DR_VectorGraphics_destroy(Tango3DR_PolygonArray* graphics);

#ifdef __cplusplus
}
#endif

#endif  // TANGO_3D_RECONSTRUCTION_API_TANGO_3D_RECONSTRUCTION_API_H_
