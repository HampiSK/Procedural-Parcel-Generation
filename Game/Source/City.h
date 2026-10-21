#pragma once

#include <array>
#include <cfloat>
#include <cstdint>
#include <span>

#include <raylib.h>

// Samples along a curved road, 16 pieces
constexpr int CURVE_POINTS = 17;

// Capacities of a city
constexpr int MAX_NODES = 1024;
constexpr int MAX_ROADS = 1024;
constexpr int MAX_HALFEDGES = 2 * MAX_ROADS;
constexpr int MAX_PARCELS = 256;
constexpr int MAX_PARCEL_POINTS = 32768;
constexpr int MAX_PARCEL_CORNERS = 1024;
constexpr int MAX_PARCEL_INDICES = 3 * MAX_PARCEL_POINTS;
constexpr int MAX_LOTS = 2048;
constexpr int MAX_LOT_POINTS = 65536;
constexpr int MAX_HOUSE_PROPS = 8;

// The parcel types, each one cuts a parcel into blocks and lots its own way
enum SubdivisionStyle
{
    SubdivisionStyle_None = -1, SubdivisionStyle_Grid, SubdivisionStyle_Organic, SubdivisionStyle_Radial, SubdivisionStyle_Random, SubdivisionStyle_Count
};

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
    // The kind tells whether node or road is set, t is the curve parameter along the road
    AnchorKind kind = AnchorKind::Free;
    int node = 0;
    int road = 0;
    float t = 0.0f;
    Vector2 position = { 0.0f, 0.0f };
};

// Axis aligned box, empty while min is above max
struct Box
{
    Vector2 min = { FLT_MAX, FLT_MAX };
    Vector2 max = { -FLT_MAX, -FLT_MAX };
};

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

// The type of a parcel and how it was generated last
struct ParcelAttributes
{
    SubdivisionStyle style = SubdivisionStyle_Grid;
    ParcelSettings settings;

    // The generation that generated the parcel last, all streets it made carry it. Zero for a parcel that was never generated
    unsigned block = 0;

    // The middle of the pattern it was generated in, radial streets and lots go around it
    Vector2 center = { 0.0f, 0.0f };
};

// A road between two intersections with its samples and box
struct Road
{
    int from = 0;
    int to = 0;
    RoadAttributes attributes;
    Polyline line;
    Box box;
};

// One side of a road. Halfedge 2r runs along road r from its first node and 2r + 1 runs back, the face on the left of a halfedge is a
// parcel or the outside
struct Halfedge
{
    // The next halfedge around the face, the next road clockwise around the node it reaches, and the parcel of the face, -1 outside
    int next = 0;
    int parcel = -1;

    // The parcel on this side as it was last, so that parcels keep their type while the roads around them change
    ParcelAttributes attributes;
    bool hasAttributes = false;

    // Set when the road is new or changed, the parcels it bounds are generated again
    bool fresh = true;
};

// A face of the roads, with what drawing and hit tests need. Its corners follow the roads inset a little, so dead ends are slits in it
struct Parcel
{
    ParcelAttributes attributes;

    // One halfedge around it, and its ranges in City::parcelPoints and City::parcelIndices, three indices per triangle
    int halfedge = 0;
    int firstPoint = 0;
    int pointCount = 0;
    int firstIndex = 0;
    int indexCount = 0;

    float area = 0.0f;
    Box box;
    Vector2 probe = { 0.0f, 0.0f };

    // Set when it is bounded by a fresh halfedge
    bool changed = false;
};

// The things around a house
enum PropKind
{
    PropKind_Tree, PropKind_Bush, PropKind_Pool, PropKind_Shed, PropKind_Car
};

// A thing around a house, trees and bushes are disks of radius halfSize.x, the others boxes turned like the house
struct Prop
{
    PropKind kind = PropKind_Tree;
    Vector2 position = { 0.0f, 0.0f };
    Vector2 halfSize = { 0.0f, 0.0f };
    int color = 0;
};

// One house on a lot seen from above: a box with a gable roof that faces a road, a driveway out to the middle of the road and the
// things around it
struct House
{
    // The seed it was drawn from, a lot where no house fits keeps it too
    unsigned seed = 0;
    bool built = false;

    // The middle of the house, the direction along its front, half its width along the front and half its depth
    Vector2 center = { 0.0f, 0.0f };
    Vector2 axis = { 1.0f, 0.0f };
    Vector2 halfSize = { 0.0f, 0.0f };
    int roof = 0;

    // The middle line of the driveway, without one both ends are the same
    Vector2 drivewayStart = { 0.0f, 0.0f };
    Vector2 drivewayEnd = { 0.0f, 0.0f };

    std::array<Prop, MAX_HOUSE_PROPS> props = {};
    int propCount = 0;
};

// A polygon inside a parcel with one house, its corners are in City::lotPoints and whether its sides run along a road in
// City::lotStreets
struct Lot
{
    int firstPoint = 0;
    int pointCount = 0;
    Box box;
    Vector2 probe = { 0.0f, 0.0f };
    House house; // Just buy a house
};

// The parcels of a block and the streets between them
struct Selection
{
    std::array<int, MAX_PARCELS> parcels = {};
    std::array<int, MAX_ROADS> roads = {};
    int parcelCount = 0;
    int roadCount = 0;
};

// Roads, the parcels between them and the lots inside the parcels. Read the members freely, change them through the methods
class City
{
public:
    std::array<Vector2, MAX_NODES> nodes = {};
    std::array<Road, MAX_ROADS> roads = {};
    std::array<Halfedge, MAX_HALFEDGES> halfedges = {};
    int nodeCount = 0;
    int roadCount = 0;

    // From the largest to the smallest area, so a parcel inside another one is drawn on top. Traced again after every change of the roads
    std::array<Parcel, MAX_PARCELS> parcels = {};
    std::array<Vector2, MAX_PARCEL_POINTS> parcelPoints = {};
    std::array<uint16_t, MAX_PARCEL_INDICES> parcelIndices = {};
    int parcelCount = 0;

    std::array<Lot, MAX_LOTS> lots = {};
    std::array<Vector2, MAX_LOT_POINTS> lotPoints = {};
    std::array<bool, MAX_LOT_POINTS> lotStreets = {};
    int lotCount = 0;
    int lotPointCount = 0;

    // Changes whenever the parcels are traced, to tell when parcel indices are out of date
    unsigned meshVersion = 0;

    // Parcels that appear or change when roads change are generated, switching it off leaves them as they are
    bool autoGenerate = true;

    // The settings of every parcel type by SubdivisionStyle, and the type of parcels that roads enclose for the first time
    std::array<ParcelSettings, SubdivisionStyle_Count> parcelSettings = {};
    SubdivisionStyle newParcelStyle = SubdivisionStyle_Grid;

    // The first capacity that something did not fit into since this was cleared, null while everything fit. What did not fit was left
    // out, the editor shows it until the next click
    const char *error = nullptr;

    City() = default;
    City(const City &) = delete;
    City &operator=(const City &) = delete;

    // Closest intersection or point along a road within the radius, intersections win, a free point when nothing is near
    RoadAnchor Snap(Vector2 point, float radius) const;
    bool FindNode(Vector2 point, float radius, int &node) const;

    // Closest road within reach of the point, the reach is at least half the road width
    bool FindRoad(Vector2 point, float radius, int &road) const;

    // Box of the intersections, or the area that is shown while there are no roads
    Box Bounds() const;

    // A road needs some length and a curve that does not fold or bend tighter than it is wide, and must not run along another road,
    // which sets alongRoad
    bool CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road, bool &alongRoad) const;

    // Adds the road, splits every road it crosses or ends on and generates the parcels it changes
    bool AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road);

    // An intersection can move when its roads stay valid and keep clear of the others, the parcels beside them are generated again
    bool CanMoveNode(int node, Vector2 position) const;
    bool MoveNode(int node, Vector2 position);

    // Moves every intersection of the list that can move to its position, then generates the parcels beside them once. False when none
    // moved
    bool MoveNodes(std::span<const int> list, std::span<const Vector2> positions);

    // Removes the road at point, the parcels on its sides merge. False when no road was hit
    bool RemoveRoadAt(Vector2 point, float radius);

    // Gives the parcel at point, or its whole block without the streets it was cut up with, a type and generates it again
    bool ApplyParcelTypeAt(Vector2 point, SubdivisionStyle style, bool wholeBlock);

    // Draws a house at random from the seed onto the lot, a lot where none fits gets trees and bushes
    void BuildHouse(int lot, unsigned seed);

    // The smallest parcel that holds the point, valid until the roads change
    bool ParcelAt(Vector2 point, int &parcel) const;

    // The lot that holds the point, valid until the lots change
    bool LotAt(Vector2 point, int &lot) const;

    // The parcel at point, or with wholeBlock all parcels its generation cut up as far as its streets join them, and those streets
    bool FindBlock(Vector2 point, bool wholeBlock, Selection &selection) const;

    std::span<const Vector2> ParcelOutline(int parcel) const;
    std::span<const Vector2> LotOutline(int lot) const;

    void Draw(const Camera2D &camera) const;

    // Previews of what a click would change
    void DrawRoadHighlight(int road, Color color) const;
    void DrawMovedNode(int node, Vector2 position, Color color) const;
    void DrawSelection(const Selection &selection, Color color) const;
    void DrawLot(int lot, Color color) const;

private:
    // Shorter roads are refused, intersections closer than the merge distance are one
    static constexpr float MIN_ROAD_LENGTH = 1.0f;
    static constexpr float MERGE_DISTANCE = 0.5f;

    // Most roads that can end at an intersection that is moved
    static constexpr int MAX_MOVED_ROADS = 16;

    // Width of the widest road at each node and the number of roads there
    std::array<float, MAX_NODES> nodeWidth = {};
    std::array<int, MAX_NODES> nodeDegree = {};

    // The number of the next generation, drawn roads have zero
    unsigned nextGeneration = 1;

    // Makes the message the error unless there is one, false for what did not fit
    bool Overflow(const char *message);

    // ---- Roads

    int Origin(int halfedge) const;

    // Sample i along a halfedge, in its direction
    Vector2 HalfedgePoint(int halfedge, int i) const;

    // Adds the road and splits the roads it crosses, without tracing the parcels again. A piece inside a parcel takes its type
    bool InsertRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road);
    void AppendRoad(int from, int to, const RoadAttributes &attributes);
    void RefreshRoad(int road);

    // The intersection at a point: one closer than the merge distance, else a new one that splits the road there or stands free
    int NodeAt(Vector2 point);

    // Cuts a road at t at a new intersection or one that is already there, the road keeps its index as the first half
    int SplitRoad(int road, float t);

    // Removes the roads and marks the parcels on their sides changed. Intersections that stop being intersections go, indices change
    void RemoveRoads(std::span<const int> list);

    // A node between just two straight roads alike that continue each other is not needed, the two roads become one
    void MergeStraightRoads(int node);

    // Counts the roads and finds the widest one at every node, then drops the nodes no road ends at
    void RefreshNodes();

    // Removes the roads of the generation that end in nothing or reach no road of another generation
    void RemoveHanging(unsigned generation);

    // The roads of the node as they would run with it at position, none for too many roads
    int PreviewMovedNode(int node, Vector2 position, std::array<Polyline, MAX_MOVED_ROADS> &lines, std::array<int, MAX_MOVED_ROADS> &incident, float &width) const;

    // ---- Parcels

    // Links every halfedge to the next one around its face
    void Link();

    // Links the halfedges and makes every face of positive area a parcel, from the type of its halfedges
    void Trace();

    float LoopArea(int start) const;
    int LoopOutline(int start, std::span<Vector2> out) const;
    void MarkLoop(int start);
    void SetAttributes(int parcel, const ParcelAttributes &attributes);
    void ClearFresh();

    // Whether the road is a street of the block between two parcels of the block
    bool JoinsBlock(int road, unsigned block) const;

    // Traces the parcels, drops the lots of the changed ones and generates them again
    void Rebuild(bool withStreets);

    // Generates the parcel at the point: the streets of its type when it is larger than a block, then lots in every block
    void GenerateAt(Vector2 point, bool withStreets);

    // Lays the street pattern of the parcel, cut where it crosses roads, as roads of the generation
    void AddStreets(int parcel, unsigned generation);

    // Lays a street of the generation from a to b, its ends snap to the roads there. False when it cannot be a road
    bool LayStreet(Vector2 a, Vector2 b, RoadAttributes road, unsigned generation);

    // Lays the pieces of the street from a to b that lie in the parcel, cut where it crosses roads, or without lay only tells whether it
    // could. True when there is a piece in the parcel and every one can be laid
    bool LayStreetIn(int parcel, Vector2 a, Vector2 b, const RoadAttributes &road, unsigned generation, bool lay);

    // Merges the blocks of the generation narrower than minBlockSize into a neighbor and cuts the ones longer than maxBlockSize in two with
    // a street, until every block fits or a round changes nothing. A long block that cannot be cut across merges too, so that the merged
    // block can. Where the roads leave no way, a block stays long rather than a sliver
    void FitBlocks(unsigned generation);

    // One round of FitBlocks each, true when blocks changed. Every block merges at most once per round
    bool MergeBlocks(unsigned generation, unsigned seed);
    bool SplitLongBlocks(unsigned generation, unsigned seed);

    // Cuts the parcel into lots that are on a road, every lot gets a house. Organic lots can need streets, which are laid without
    // tracing the parcels again
    void FillLots(int parcel, unsigned seed);
    void AddLot(std::span<const Vector2> points, std::span<const bool> streets, unsigned seed);

    // Whether a road runs through the polygon
    bool RoadThrough(std::span<const Vector2> polygon) const;

    // ---- Drawing

    void DrawParcel(int parcel, Color color) const;

    // A disk as wide as the widest road of an intersection hides gaps at joins and caps dead ends
    void DrawDisks(Box view, float worldPerPixel, float extra, float minRadiusPixels, Color color) const;
};

// The toolbar and the tools that edit a city, the city to edit is passed in every frame
class Editor
{
public:
    Editor();
    ~Editor();
    Editor(const Editor &) = delete;
    Editor &operator=(const Editor &) = delete;

    // Previews of the selected tool in the world, handles the clicks there. Call it in BeginMode2D with the camera, before Draw
    void DrawWorld(City &city, const Camera2D &camera);

    // Toolbar and status line in the window, handles the clicks on the toolbar. Call it outside of BeginMode2D, after DrawWorld
    void Draw(const City &city);

private:
    // The toolbar items in the order they are shown: the buttons, then the options of the selected button's tool above them
    enum Item
    {
        Item_None = -1, Item_Road, Item_Remove, Item_Move, Item_ParcelGrid, Item_ParcelOrganic, Item_ParcelRadial, Item_ParcelRandom, Item_Curved, Item_OneParcel, Item_OneLot, Item_Count
    };

    enum class Tool
    {
        None, Road, Remove, Move, Parcel
    };

    struct ToolbarItem
    {
        Tool tool = Tool::Road;
        const char *label = "";
        const char *iconPath = "";
        const char *hint = "";
        bool isOption = false;
    };

    static constexpr ToolbarItem ITEMS[Item_Count] =
    {
        ToolbarItem { Tool::Road, "Road", "../Resource/road.png", "Click to start a road, snap to a road or an intersection to connect", false },
        ToolbarItem { Tool::Remove, "Remove", "../Resource/remove.png", "Click a road to remove it", false },
        ToolbarItem { Tool::Move, "Move", "../Resource/move.png", "Drag an intersection to move it", false },
        ToolbarItem { Tool::Parcel, "Grid", "../Resource/grid.png", "Click a parcel to change it to a grid", false },
        ToolbarItem { Tool::Parcel, "Organic", "../Resource/organic.png", "Click a parcel to change it to organic", false },
        ToolbarItem { Tool::Parcel, "Radial", "../Resource/radial.png", "Click a parcel to change it to radial", false },
        ToolbarItem { Tool::Parcel, "Random", "../Resource/random.png", "Click a parcel to change it to random", false },
        ToolbarItem { Tool::Road, "Curved", "../Resource/curved.png", "", true },
        ToolbarItem { Tool::Parcel, "One Parcel", "../Resource/one_parcel.png", "", true },
        ToolbarItem { Tool::Parcel, "One Lot", "../Resource/one_lot.png", "", true },
    };

    Texture2D icons[Item_Count] = {};

    Item selectedButton = Item_None;
    Item selectedOption = Item_None;

    // True while the mouse is over the toolbar, the tools ignore clicks then
    bool mouseOverToolbar = false;

    // The tool the state below belongs to, it starts over when another tool is picked
    Tool activeTool = Tool::None;

    // The road being drawn: its start, and the end of a curved road while its bend is placed
    RoadAnchor roadStart;
    RoadAnchor roadEnd;
    bool hasStart = false;
    bool hasEnd = false;

    // The intersection being dragged, -1 while none is
    int movedNode = -1;

    // Where the right button went down, a right click without dragging cancels, dragging pans the camera
    Vector2 rightPress = { 0.0f, 0.0f };

    // What the tool said in DrawWorld, Draw shows it in the status line unless the city has an error
    const char *toolStatus = "";

    // Draws one toolbar item with the icon above the label or left of it, returns true when it was clicked
    bool DrawItem(Rectangle bounds, Texture2D icon, const char *label, bool selected, bool iconAbove);

    // The tools, each previews what a click would do and does it. The road and move tools tell what is wrong in status
    void UseRoad(City &city, Vector2 mouse, float snapRadius, float worldPerPixel, bool click, const char *&status);
    void UseRemove(City &city, Vector2 mouse, float snapRadius, bool click);
    void UseMove(City &city, Vector2 mouse, float snapRadius, float worldPerPixel, bool click, const char *&status);
    void UseParcel(City &city, Vector2 mouse, bool click, const char *&status);
};
