#pragma once

#include <array>
#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <raylib.h>

// Samples along a curved road, 16 pieces
constexpr int CURVE_POINTS = 17;

// The parcel types, each one cuts a parcel into blocks and lots its own way
enum class SubdivisionStyle
{
    Grid, Organic, Radial, Random
};
constexpr int SUBDIVISION_STYLE_COUNT = 4;

// What the end of a road is attached to: nothing, an intersection or a point along a road
enum class AnchorKind
{
    Free, Node, Road
};

// Samples along a road, a straight road keeps only its two ends
struct Polyline
{
    std::array<Vector2, CURVE_POINTS> points = {};
    int count = 0;
};

// How wide a road is, which generation made it and how it bends
struct RoadAttributes
{
    float width = 2.0f;

    // Roads made by generating parcels carry the number of that generation, drawn roads have zero
    unsigned generation = 0;

    // The inner control points of a curved road
    bool curved = false;
    std::array<Vector2, 2> controls = {};
};

// Where a road can start or end: a free point, an intersection or a point along a road
struct RoadAnchor
{
    // The kind tells whether the node or the segment is set
    AnchorKind kind = AnchorKind::Free;
    size_t node = 0;
    size_t segment = 0;

    // Curve parameter along the segment, from its first to its second node, and where the anchor is
    float t = 0.0f;
    Vector2 position = { 0.0f, 0.0f };
};

// Axis aligned box, empty while min is above max
struct Box
{
    Vector2 min = { FLT_MAX, FLT_MAX };
    Vector2 max = { -FLT_MAX, -FLT_MAX };
};

// One road between two intersections, with its geometry cached for drawing and hit tests
struct RoadSegment
{
    // The intersections at its ends and how the road is built
    size_t from = 0;
    size_t to = 0;
    RoadAttributes road;

    // Samples along the middle of the road, and its outline as a strip with a left and a right point per sample
    Polyline line;
    std::array<Vector2, 2 * CURVE_POINTS> ribbon = {};
    Box box;
};

// ---- Road graph

// Planar graph of roads, the source of truth for the city layout. Every intersection has a road
class RoadGraph
{
public:
    // Read them freely, change them through the methods so that the cached geometry follows
    std::vector<Vector2> nodes;
    std::vector<RoadSegment> segments;

    // Closest intersection or point along a road within the radius, intersections win, a free point when nothing is near
    RoadAnchor Snap(Vector2 point, float radius) const;

    // The intersection closest to the point within the radius, on a tie the later one
    bool FindNode(Vector2 point, float radius, size_t &node) const;

    // Closest road within reach of the point, the reach is at least half the road width
    bool FindSegment(Vector2 point, float radius, size_t &index) const;

    // Box of the intersections, or the area that is shown while there are no roads
    Box Bounds() const;

    // A road needs some length and a curve that does not cross itself, fold back or bend tighter than the road is wide
    bool CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road) const;

    // Whether a stretch of the road would be nearly parallel to a road that is there and so close that the two overlap
    bool RunsAlongRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road) const;

    // Adds the road and splits every road it crosses or ends on, false when the road is refused
    bool AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road);

    // An intersection can move when its roads stay valid roads that keep clear of the others, and it keeps clear of them too
    bool CanMoveNode(size_t node, Vector2 position) const;

    // Moves the intersection when it can move there, its roads follow
    bool MoveNode(size_t node, Vector2 position);

    // The roads of the intersection as they would run with it at position
    void DrawMovedNode(size_t node, Vector2 position, Color color) const;

    // Removes the segments, false when there are none. Intersections that stop being intersections go, indices change
    bool RemoveSegments(std::vector<size_t> indices);

    // Removes the roads of the generation that reach no road of another one, and with deadEnds the ones that end in nothing
    bool RemoveHanging(unsigned generation, bool deadEnds);

    // Appends the curve parameters where the line crosses roads of another generation
    void Crossings(const Polyline &line, unsigned generation, std::vector<float> &params) const;

    // Three passes merge intersections without seams: a lighter edge around every road, the roads, a dashed middle line on wide roads
    void Draw(Box view, float worldPerPixel) const;

    // Paints over one road, to preview what a click would remove
    void DrawSegmentHighlight(size_t index, Color color) const;

    // Every bounded face of the roads as a simple outline, the corners of all outlines one after another in points and the corner
    // count of each outline in sizes. Halfedge 2s runs along segment s from its first node and 2s + 1 runs back, the face on the
    // left of a halfedge goes on with the next road clockwise around the node it reaches
    void TraceParcels(std::vector<Vector2> &points, std::vector<size_t> &sizes) const;

private:
    // Shorter roads are refused
    static constexpr float MIN_ROAD_LENGTH = 1.0f;

    // Intersections closer to each other than this are one
    static constexpr float MERGE_DISTANCE = 0.5f;

    // Most roads that can end at an intersection that is moved, with more it stays where it is
    static constexpr size_t MAX_MOVED_ROADS = 16;

    // Width of the widest road at each node and the number of roads there, refreshed after every change
    std::vector<float> nodeWidth;
    std::vector<int> nodeDegree;

    // A disk as wide as the widest road of an intersection hides gaps at joins and caps dead ends, small disks get fewer corners
    void DrawDisks(Box view, float worldPerPixel, float extra, float minRadiusPixels, Color color) const;

    // The node a halfedge leaves
    size_t Origin(size_t halfedge) const;

    // The shoelace sum of the halfedges of a walk from index from to index to, over a closed walk it is the area inside
    float WalkArea(const std::vector<size_t> &walk, size_t from, size_t to) const;

    // Makes the walk around a face a simple outline: dead ends, walked out and back, go piece by piece, and of a walk that comes back to
    // a node it passed the part with the larger area stays. The other part goes to parts, followed by its length
    void SimplifyWalk(std::vector<size_t> &walk, std::vector<size_t> &parts) const;

    // The roads of the node as they would run with it at position, their segments and the widest width, none for too many roads
    size_t PreviewMovedNode(size_t node, Vector2 position, std::array<Polyline, MAX_MOVED_ROADS> &lines, std::array<size_t, MAX_MOVED_ROADS> &incident, float &width) const;

    // The intersection at a point: one closer than the merge distance, else a new one that splits the road the point is on or stands free
    size_t NodeAt(Vector2 point);

    // Appends the segment and builds its samples, outline and box
    void AddSegment(const RoadSegment &segment);

    // Cuts a segment at t at a new intersection or at one that is already there, the segment keeps its index as the first half
    size_t SplitSegment(size_t index, float t);

    // The segment between the ends of the segment that runs along it the whole way, SIZE_MAX when there is none
    size_t SameRoadBetween(const RoadSegment &segment) const;

    // Rebuilds the samples, outline and box of a segment, a joint is offset along the bisector and sharp ones are cut off
    void RefreshSegment(size_t index);

    // Counts the roads and finds the widest one at every node, then drops the nodes no road ends at and numbers the rest again
    void RefreshNodes();

    // A node between just two straight roads alike that continue each other is not needed, the two roads become one
    void MergeStraightRoads(size_t node);
};

// ---- City

// The sizes a parcel type cuts a parcel into
struct ParcelSettings
{
    // Lots are split while they are larger or wider than the maximums, no split leaves a lot smaller or narrower than the minimums
    float minParcelArea = 100.0f;
    float maxParcelArea = 500.0f;
    float minParcelWidth = 5.0f;
    float maxParcelWidth = 20.0f;

    // How much chance changes the splits, with zero the best split is taken every time
    float splitIrregularity = 0.0f;

    // Sides of the blocks a parcel longer than a block is cut into by the streets of its type first
    float minBlockSize = 25.0f;
    float maxBlockSize = 125.0f;
};

// Defined in City.cpp
struct ParcelMesh;
struct LotShape;
struct Street;

// Roads, the parcels between them and the lots inside the parcels
class City
{
public:
    // The roads, the parcels are the faces between them. Change them through the city so that the parcels follow
    RoadGraph graph;

    // Changes whenever the parcels are rebuilt, to tell when indices of parcels are out of date. It counts over all cities, so no two
    // cities with parcels share a version
    unsigned meshVersion = 0;

    // Parcels that appear or change when roads are added or removed are generated like that, switching it off leaves them as they are
    bool autoGenerate = true;

    // The settings of every parcel type by SubdivisionStyle, a parcel is generated with the settings of its type at that time
    std::array<ParcelSettings, SUBDIVISION_STYLE_COUNT> parcelSettings = {};

    // The type of the parcels that roads enclose for the first time
    SubdivisionStyle newParcelStyle = SubdivisionStyle::Grid;

    // A city without roads, it owns its parcels and is not copied
    City();
    ~City();
    City(const City &) = delete;
    City &operator=(const City &) = delete;

    // Draws what the camera sees
    void Draw(const Camera2D &camera) const;

    // A road can be added when it is long enough, does not fold and does not run alongside another road, which sets alongRoad
    bool CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road, bool &alongRoad) const;

    // Adds the road when it can be added, the parcels it changes are generated again
    bool AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road);

    // Moves an intersection, the parcels beside its roads are generated again
    bool MoveNode(size_t node, Vector2 position);

    // Removes the road at point, the parcels on its sides merge or go when the other side is outside. False when no road was hit
    bool RemoveRoadAt(Vector2 point, float radius);

    // Gives the parcel at point, or its whole block, a type and generates it again. A whole block loses the streets it was cut up with
    // first, so it is one parcel again. False when no parcel was hit
    bool ApplyParcelTypeAt(Vector2 point, SubdivisionStyle style, bool wholeBlock);

    // The smallest parcel that holds the point, a parcel inside another parcel wins. The index is valid until the roads change
    bool ParcelAt(Vector2 point, size_t &index) const;

    // The parcel at point, or with wholeBlock all parcels its generation cut up as far as its streets join them, and those streets
    bool FindBlock(Vector2 point, bool wholeBlock, std::vector<size_t> &parcels, std::vector<size_t> &streets) const;

    // Paints over parcels by their indices, to preview what a click would change
    void DrawParcels(const std::vector<size_t> &shapes, Color color) const;

private:
    // The current parcels and the ones from before the last rebuild, both are empty while no road encloses anything
    std::unique_ptr<ParcelMesh> mesh;
    std::unique_ptr<ParcelMesh> oldMesh;

    // Lots stay while a road runs along them and none through them, and the parcel around them is there. The corners of all lots
    // are stored one after another
    std::vector<Vector2> lotPoints;
    std::vector<LotShape> lotShapes;

    // The number of the next generation, the roads that are drawn have zero
    unsigned nextGeneration = 1;

    // The parcels on both sides of the middle of a road, false when a side has none
    bool StreetSides(size_t street, size_t &left, size_t &right) const;

    // The corners of a lot
    std::span<const Vector2> LotPoints(size_t lot) const;

    // Builds the parcels from the roads again, a parcel takes the settings of the old parcel at its inside point
    void BuildParcelMesh();

    // Builds the parcels again after the roads changed and generates the ones that changed, the one at freed and the ones that lost lots
    void RebuildParcels(bool withStreets, std::optional<Vector2> freed);

    // Generates the parcel at the point: the streets of its type when it is larger than a block, then lots in every block with the
    // streets the lots need. All of it is marked with a new generation
    void GenerateAt(Vector2 point, bool withStreets);

    // Generated streets become roads of the generation that join the roads they end on. They are cut where they cross another road,
    // and the pieces outside the blocks of the generation, like inside a closed road, are left out
    bool AddStreets(const std::vector<Street> &streets, unsigned generation);

    // Adds a lot, its corners go after the corners of all other lots
    void AddLot(std::span<const Vector2> points);

    // Keeps the marked lots in their order
    void KeepLots(const std::vector<char> &keep);

    // Removes the lots in the marked parcels, a lot belongs to the smallest parcel that holds its inside point
    void RemoveLotsIn(const std::vector<char> &parcels);

    // Removes the lots outside every parcel and the lots that are not on a road or in their parcel any more, marks the parcels that lost lots
    std::vector<char> PruneLots();

    // Gives a parcel new lots without streets in place of the ones it has, roads that end in it cut it first
    void FillLots(size_t index, unsigned seed);
};

// ---- Editor

// The toolbar and the tools that edit a city, the city to edit is passed in every frame
class Editor
{
public:
    // Loads the icons, so the window has to be open. The editor owns them and is not copied
    Editor();
    ~Editor();
    Editor(const Editor &) = delete;
    Editor &operator=(const Editor &) = delete;

    // Previews and markers of the selected tool, handles the clicks in the world too, call inside the 2D camera mode
    void DrawWorld(City &city, const Camera2D &camera);

    // Toolbar and status line, handles the toolbar clicks too, call in screen space after DrawWorld
    void Draw();

private:
    // The buttons of the toolbar and the options above them, in the order they are shown
    enum ButtonKind
    {
        None = -1, Road, Remove, Move, ParcelGrid, ParcelOrganic, ParcelRadial, ParcelRandom, _Button_Count
    };
    enum OptionKind
    {
        Curved, OneParcel, _Option_Count
    };
    static_assert(ParcelRandom - ParcelGrid == (int)SubdivisionStyle::Random, "The parcel buttons follow SubdivisionStyle");

    // The icons of the items, which options are on and the selected button
    std::array<Texture2D, ButtonKind::_Button_Count> buttonIcons = {};
    std::array<Texture2D, OptionKind::_Option_Count> optionIcons = {};
    std::array<bool, OptionKind::_Option_Count> optionEnabled = {};
    ButtonKind selectedButton = ButtonKind::None;

    // Written by Draw and read by DrawWorld in the next frame, the world ignores clicks while the mouse is over the toolbar
    bool mouseOverToolbar = false;

    // Written by DrawWorld and read by Draw in the same frame
    const char *status = "";

    // Clicks done in the current action: the road start and its bend point, or 1 while an intersection is dragged
    int step = 0;
    RoadAnchor start;
    Vector2 bend = { 0.0f, 0.0f };
    size_t dragNode = 0;

    // Where the right button went down, letting it go at the same place cancels the action
    Vector2 rightPress = { 0.0f, 0.0f };

    // Parcels a click would change, looked up again only when the parcel under the mouse, the option or the parcels change
    std::vector<size_t> hoverShapes;
    size_t hoverParcel = SIZE_MAX;
    unsigned hoverVersion = 0;
    bool hoverWhole = true;

    // Draws one toolbar item and returns true when it was clicked
    bool DoItem(Rectangle bounds, Rectangle iconArea, Vector2 labelPosition, Texture2D icon, int fontSize, const char *label, bool selected);
};