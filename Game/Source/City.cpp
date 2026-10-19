#include "City.h"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <raymath.h>

// ---- Types

// Every road is a cubic Bezier from its first to its last point
struct Bezier
{
    std::array<Vector2, 4> points = {};
};

struct ParcelAttributes
{
    // The type of the parcel and the settings of the type it was generated with last
    SubdivisionStyle style = SubdivisionStyle::Grid;
    ParcelSettings settings;

    // The generation that generated the parcel last, all streets it made carry it. Zero for a parcel that was never generated
    unsigned block = 0;

    // The middle of the pattern it was generated in, radial streets and lots go around it
    Vector2 center = { 0.0f, 0.0f };
};

// A generated street from a to b, curved when its road is
struct Street
{
    Vector2 a = { 0.0f, 0.0f };
    Vector2 b = { 0.0f, 0.0f };
    RoadAttributes road;
};

// Same seed, same layout
struct Random
{
    unsigned state = 1;

    float Next();
};

// Where a side of a polygon crosses a line, t is the distance along the line. A crossing moved onto a corner has the side starting there
struct Crossing
{
    size_t side = 0;
    Vector2 point = { 0.0f, 0.0f };
    float t = 0.0f;
    bool corner = false;
};

// Where a line crosses a road: the curve parameter along the line, the road and the curve parameter along the road
struct Hit
{
    float along = 0.0f;
    size_t segment = 0;
    float t = 0.0f;
};

// Lowest and highest projection of points on an axis
struct Interval
{
    float low = 0.0f;
    float high = 0.0f;
};

// Smallest rectangle around a polygon with one side along a side of the polygon, the smallest of all for a convex polygon
struct OrientedBox
{
    Vector2 axisLong = { 1.0f, 0.0f };
    Vector2 axisShort = { 0.0f, 1.0f };
    float length = 0.0f;
    float width = 0.0f;
};

// ---- Lots after Chen, Song and Ortner, Hierarchical Co-generation of Parcels and Streets in Urban Modeling (CGF 2024), and Vanegas
// et al., Procedural Generation of Parcels in Urban Modeling (CGF 2012). A block is split in two along the best candidate line until
// its lots are small enough, a split that would leave a part with too little street becomes a street, the I-shaped street of Chen et al

// A polygon while it is split, street[i] is set when the side from corner i to the next one runs along a street
struct Cell
{
    std::vector<Vector2> points;
    std::vector<char> street;

    void Add(Vector2 point, char isStreet);
};

// The two parts of a split, the chord between them, whether it becomes a street, and how good the split is
struct Candidate
{
    std::array<Cell, 2> parts;
    std::array<Crossing, 2> ends;
    Street chord;
    bool street = false;
    float quality = -FLT_MAX;
};

// A candidate line for a split
struct Line
{
    Vector2 origin = { 0.0f, 0.0f };
    Vector2 direction = { 1.0f, 0.0f };
};

class LotSubdivider
{
public:
    LotSubdivider(const ParcelAttributes &parcelAttributes, unsigned seed);

    // Appends the lots of a simple block and the streets they need, every side of the block is a road and roads that end inside it
    // are in roads. The corners of the lots go one after another into lotPoints and the corner count of each lot into lotSizes
    void Run(std::span<const Vector2> outline, std::span<const Polyline> roads, std::vector<Vector2> &lotPoints, std::vector<size_t> &lotSizes, std::vector<Street> &streets);

private:
    // Corners closer than this are one
    static constexpr float SAME_POINT = 0.001f;
    static constexpr int MAX_LINES = 20;
    static constexpr int BEND_POINTS = 7;

    ParcelAttributes attributes;
    Random random;

    // Buffers that every split uses again, so that splitting allocates nothing once they have grown
    Candidate best;
    Candidate candidate;
    std::vector<Crossing> crossings;
    std::vector<Vector2> corners;
    std::vector<float> cornerAngles;
    std::vector<float> sideLengths;

    // Candidates for the line of a split, spread over the middle of the cell, box is its oriented box
    int CollectLines(std::span<const Vector2> points, const OrientedBox &box, std::array<Line, MAX_LINES> &lines);

    // Splits the cell along the best chord of the candidate lines and adds the chord to the streets when it has to be one
    bool Split(const Cell &cell, std::array<Cell, 2> &parts, std::vector<Street> &streets);

    // Cuts the cell along the chord between two crossings and rates the split by Equation 2 of Chen et al. The chord becomes a street,
    // with an end on a street, when the cell is wide or a part would have less street than a lot is wide
    bool Evaluate(const Cell &cell, Crossing a, Crossing b, bool wide, Candidate &split);

    // Makes a road that lies in the cell with one end on a side a narrow slit in it, streets on both sides, so that no lot crosses it
    void AddSlit(Cell &cell, const Polyline &road);

    // The parts of the cell on both sides of the chord from a to b, a is on an earlier side than b, a curved chord has its corners in bend
    void Cut(const Cell &cell, const Crossing &a, const Crossing &b, std::span<const Vector2> bend, char chordStreet, std::array<Cell, 2> &parts);

    // The chord from a to b bent into a gentle curve inside the polygon, as corners from a to b and as a road. False when none fits
    bool Bend(std::span<const Vector2> polygon, Vector2 a, Vector2 b, std::array<Vector2, BEND_POINTS + 2> &line, RoadAttributes &road);
};

// ---- Parcels

// The columns and rows of the cells of a grid that a box overlaps
struct GridRange
{
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
};

// Boxes sorted into the cells of a uniform grid, finds the boxes near a query without testing every box. The boxes of all cells are
// stored one after another, the ones of cell c from cellStart[c] to cellStart[c + 1]
class BoxGrid
{
public:
    void Build(std::span<const Box> boxes);

    GridRange Range(Box box) const;

    // The boxes that overlap a cell, or the cell of a point
    std::span<const size_t> At(int x, int y) const;
    std::span<const size_t> At(Vector2 p) const;

private:
    Box bounds;
    float cellSize = 1.0f;
    int columns = 1;
    std::vector<size_t> cellStart = { 0, 0 };
    std::vector<size_t> items;

    int Index(float value, float origin) const;
};

// One parcel, a face between the roads, with what the frame loop needs, so drawing and hit tests never trace the roads
struct ParcelShape
{
    ParcelAttributes parcel;
    float area = 0.0f;
    Box box;
    Vector2 probe = { 0.0f, 0.0f };

    // Ranges in ParcelMesh::corners and in ParcelMesh::triangles, which has three corners per triangle
    size_t firstCorner = 0;
    size_t cornerCount = 0;
    size_t firstTriangle = 0;
    size_t triangleCount = 0;

    // Set when no parcel from before the rebuild had about the same place and shape
    bool changed = true;
};

// The parcels between the roads
struct ParcelMesh
{
    // From the largest to the smallest area. Parcels can lie inside other parcels, so inner parcels are drawn on top
    std::vector<ParcelShape> shapes;
    std::vector<Vector2> corners;
    std::vector<Vector2> triangles;
    BoxGrid grid;

    // Traces the parcels of the roads again, into the buffers from before
    void Build(const RoadGraph &graph);

    // The corners of a parcel by its index in shapes
    std::span<const Vector2> Outline(size_t index) const;

    // The smallest parcel that holds the point, the parcels go from the largest to the smallest
    bool Find(Vector2 point, size_t &index) const;

    void Draw(size_t index, Color color) const;
};

// A lot is a polygon inside a parcel, its corners are stored with the corners of all other lots
struct LotShape
{
    size_t first = 0;
    size_t count = 0;
    Box box;
    Vector2 probe = { 0.0f, 0.0f };
};

// ---- Editor

enum class Tool
{
    Road, Remove, Move, Parcel
};

struct ToolbarItem
{
    Tool tool = Tool::Road;
    const char *label = "";
    const char *iconPath = "";
};

// Same order as Editor::ButtonKind, the parcel buttons also in the order of SubdivisionStyle
static constexpr ToolbarItem BUTTONS[] =
{
    { Tool::Road, "Road", "../Resource/road.png" },
    { Tool::Remove, "Remove", "../Resource/remove.png" },
    { Tool::Move, "Move", "../Resource/move.png" },
    { Tool::Parcel, "Grid", "../Resource/grid.png" },
    { Tool::Parcel, "Organic", "../Resource/organic.png" },
    { Tool::Parcel, "Radial", "../Resource/radial.png" },
    { Tool::Parcel, "Random", "../Resource/random.png" },
};

// Same order as Editor::OptionKind, shown above the toolbar while a button with the same tool is selected
static constexpr ToolbarItem OPTIONS[] =
{
    { Tool::Road, "Curved", "../Resource/curved.png" },
    { Tool::Parcel, "One Parcel", "../Resource/one_parcel.png" },
};

// Counts the parcel rebuilds of all cities, so that no two meshes get the same version. Cities are edited on one thread
static unsigned gMeshVersion = 0;

// ---- Boxes

// Unit vector to the left of the way from a to b
static Vector2 leftNormal(Vector2 a, Vector2 b)
{
    return Vector2Normalize(Vector2{ a.y - b.y, b.x - a.x });
}

static Box inflate(Box box, float margin)
{
    return { box.min - Vector2{ margin, margin }, box.max + Vector2{ margin, margin } };
}

static bool overlap(Box a, Box b)
{
    return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y;
}

static bool contains(Box box, Vector2 p)
{
    return overlap(box, Box{ p, p });
}

static Box boxOf(std::span<const Vector2> points)
{
    Box box;
    for (Vector2 p : points) box = { Vector2Min(box.min, p), Vector2Max(box.max, p) };
    return box;
}

// The root of the set of x among sets where every number points at one of its set and the root at itself, halves the path walked
static size_t findSet(std::vector<size_t> &parent, size_t x)
{
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
}

// ---- Curves

// The Bezier of a road from its first to its second end
static Bezier roadBezier(Vector2 first, const RoadAttributes &road, Vector2 second)
{
    if (road.curved) return { { first, road.controls[0], road.controls[1], second } };
    return { { first, Vector2Lerp(first, second, 1.0f / 3.0f), Vector2Lerp(first, second, 2.0f / 3.0f), second } };
}

static Vector2 bezierPoint(const Bezier &curve, float t)
{
    return GetSplinePointBezierCubic(curve.points[0], curve.points[1], curve.points[2], curve.points[3], t);
}

// A road that bends towards the control point, a quadratic Bezier raised to a cubic one
static RoadAttributes bentRoad(Vector2 first, Vector2 control, Vector2 second)
{
    return { .curved = true, .controls = { Vector2Lerp(first, control, 2.0f / 3.0f), Vector2Lerp(second, control, 2.0f / 3.0f) } };
}

// Splits a Bezier at t in two, De Casteljau. The curve is a copy, so a half can be written over it
static void splitBezier(Bezier curve, float t, Bezier &left, Bezier &right)
{
    const Vector2 a = Vector2Lerp(curve.points[0], curve.points[1], t);
    const Vector2 b = Vector2Lerp(curve.points[1], curve.points[2], t);
    const Vector2 c = Vector2Lerp(curve.points[2], curve.points[3], t);
    const Vector2 ab = Vector2Lerp(a, b, t);
    const Vector2 bc = Vector2Lerp(b, c, t);
    left = { { curve.points[0], a, ab, Vector2Lerp(ab, bc, t) } };
    right = { { Vector2Lerp(ab, bc, t), bc, c, curve.points[3] } };
}

// The part of a Bezier between two curve parameters
static Bezier subBezier(const Bezier &curve, float from, float to)
{
    Bezier first;
    Bezier rest;
    splitBezier(curve, to, first, rest);
    splitBezier(first, from / fmaxf(to, 0.000001f), rest, first);
    return first;
}

// Uniform samples of a road, a straight road keeps its two ends
static Polyline roadLine(Vector2 first, const RoadAttributes &road, Vector2 second)
{
    Polyline line = { { first, second }, 2 };
    if (!road.curved) return line;

    const Bezier curve = roadBezier(first, road, second);
    line.count = CURVE_POINTS;
    for (int i = 0; i < CURVE_POINTS; i++) line.points[i] = bezierPoint(curve, (float)i / (float)(CURVE_POINTS - 1));
    return line;
}

static std::span<const Vector2> samples(const Polyline &line)
{
    return { line.points.data(), (size_t)line.count };
}

// The middle of a line, a sample or halfway between the two middle samples
static Vector2 lineMiddle(const Polyline &line)
{
    const int middle = line.count / 2;
    return line.count % 2 == 1 ? line.points[middle] : Vector2Lerp(line.points[middle - 1], line.points[middle], 0.5f);
}

// ---- Geometry

static bool insidePolygon(std::span<const Vector2> polygon, Vector2 p)
{
    return CheckCollisionPointPoly(p, polygon.data(), (int)polygon.size());
}

static float pointSegmentDistance(Vector2 p, Vector2 a, Vector2 b)
{
    const Vector2 ab = b - a;
    const float lengthSquared = Vector2LengthSqr(ab);
    return Vector2Distance(p, a + ab * (lengthSquared > 0.0f ? Clamp(Vector2DotProduct(p - a, ab) / lengthSquared, 0.0f, 1.0f) : 0.0f));
}

static float boundaryDistance(std::span<const Vector2> polygon, Vector2 p)
{
    float best = FLT_MAX;
    for (size_t i = 0; i < polygon.size(); i++) best = fminf(best, pointSegmentDistance(p, polygon[i], polygon[(i + 1) % polygon.size()]));
    return best;
}

// Where the pieces p + u r and q + v d meet, false when they are about parallel
static bool intersect(Vector2 p, Vector2 r, Vector2 q, Vector2 d, float &u, float &v)
{
    // Sine of the smallest angle at which two pieces still meet
    constexpr float PARALLEL = 0.0001f;

    const float denominator = Vector2CrossProduct(r, d);
    if (fabsf(denominator) <= PARALLEL * Vector2Length(r) * Vector2Length(d)) return false;
    u = Vector2CrossProduct(q - p, d) / denominator;
    v = Vector2CrossProduct(q - p, r) / denominator;
    return true;
}

// Whether two segments cross, touching at their ends does not count
static bool segmentsCross(Vector2 a0, Vector2 a1, Vector2 b0, Vector2 b1)
{
    constexpr float MARGIN = 0.001f;

    float u = 0.0f;
    float v = 0.0f;
    return intersect(a0, a1 - a0, b0, b1 - b0, u, v) && u > MARGIN && u < 1.0f - MARGIN && v > MARGIN && v < 1.0f - MARGIN;
}

// Distance between two segments, zero when they cross
static float segmentsDistance(Vector2 a0, Vector2 a1, Vector2 b0, Vector2 b1)
{
    if (segmentsCross(a0, a1, b0, b1)) return 0.0f;
    return fminf(fminf(pointSegmentDistance(a0, b0, b1), pointSegmentDistance(a1, b0, b1)), fminf(pointSegmentDistance(b0, a0, a1), pointSegmentDistance(b1, a0, a1)));
}

// Whether a piece of the first line crosses a piece of the second other than at their ends, a closed second line is a polygon
static bool linesCross(std::span<const Vector2> first, std::span<const Vector2> second, bool closed)
{
    for (size_t i = 0; i + 1 < first.size(); i++)
    {
        for (size_t j = 0; j + (closed ? 0 : 1) < second.size(); j++)
        {
            if (segmentsCross(first[i], first[i + 1], second[j], second[(j + 1) % second.size()])) return true;
        }
    }
    return false;
}

// Positive when the corners run counter-clockwise. Summed around the first corner, so that small parcels far out keep their sign
static float signedArea(std::span<const Vector2> polygon)
{
    float twice = 0.0f;
    for (size_t i = 1; i + 1 < polygon.size(); i++) twice += Vector2CrossProduct(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
    return 0.5f * twice;
}

// Ear clipping of a simple polygon of either winding, three corners per triangle. Without an ear the first convex corner goes anyway
static void triangulate(std::span<const Vector2> polygon, std::vector<Vector2> &out)
{
    std::vector<Vector2> ring(polygon.begin(), polygon.end());
    const float winding = signedArea(polygon) >= 0.0f ? 1.0f : -1.0f;
    while (ring.size() >= 3)
    {
        const size_t n = ring.size();
        size_t clip = n;
        bool ear = false;
        for (size_t i = 0; i < n && !ear; i++)
        {
            const Vector2 a = ring[(i + n - 1) % n];
            const Vector2 b = ring[i];
            const Vector2 c = ring[(i + 1) % n];
            if (Vector2CrossProduct(b - a, c - b) * winding <= 0.0f) continue;

            // A corner out of the box of the triangle is not in it, which is quicker to tell
            ear = true;
            const Box box = inflate(boxOf(std::array<Vector2, 3>{ a, b, c }), 0.001f);
            for (size_t j = 0; j < n && ear; j++) ear = j == i || j == (i + n - 1) % n || j == (i + 1) % n || !contains(box, ring[j]) || !CheckCollisionPointTriangle(ring[j], a, b, c);
            if (clip == n || ear) clip = i;
        }

        // A ring without convex corners has no area left
        if (clip == n) return;
        out.insert(out.end(), { ring[(clip + n - 1) % n], ring[clip], ring[(clip + 1) % n] });
        ring.erase(ring.begin() + (long)clip);
    }
}

// A point inside a simple polygon: the centroid, summed around the first corner to stay exact, or the middle of its largest triangle
static Vector2 interiorPoint(std::span<const Vector2> polygon)
{
    if (polygon.empty()) return { 0.0f, 0.0f };

    const Vector2 origin = polygon[0];
    float twiceArea = 0.0f;
    Vector2 sum = { 0.0f, 0.0f };
    for (size_t i = 0; i < polygon.size(); i++)
    {
        const Vector2 a = polygon[i] - origin;
        const Vector2 b = polygon[(i + 1) % polygon.size()] - origin;
        twiceArea += Vector2CrossProduct(a, b);
        sum += (a + b) * Vector2CrossProduct(a, b);
    }
    if (fabsf(twiceArea) > 0.000001f && insidePolygon(polygon, origin + sum / (3.0f * twiceArea))) return origin + sum / (3.0f * twiceArea);

    std::vector<Vector2> triangles;
    triangulate(polygon, triangles);
    Vector2 best = origin;
    float largest = 0.0f;
    for (size_t t = 0; t + 2 < triangles.size(); t += 3)
    {
        const float area = fabsf(Vector2CrossProduct(triangles[t + 1] - triangles[t], triangles[t + 2] - triangles[t]));
        if (area <= largest) continue;

        largest = area;
        best = (triangles[t] + triangles[t + 1] + triangles[t + 2]) / 3.0f;
    }
    return best;
}

// True for at least three corners when no two of them are at the same place and no two sides cross
static bool polygonIsSimple(std::span<const Vector2> points)
{
    constexpr float SAME_POINT = 0.001f;

    const size_t n = points.size();
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = i + 1; j < n; j++)
        {
            if (Vector2Distance(points[i], points[j]) < SAME_POINT) return false;
            if (j > i + 1 && !(i == 0 && j + 1 == n) && segmentsCross(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n])) return false;
        }
    }
    return n >= 3;
}

// Makes a polygon simple where roads cross without an intersection or run on top of each other. A corner at the place of an earlier
// one, or a side that crosses an earlier side, closes a loop, and of the loop and the rest the part with the larger area stays
static void untangle(std::vector<Vector2> &points)
{
    constexpr float SAME_POINT = 0.001f;

    std::vector<Vector2> loop;
    std::vector<Vector2> rest;
    for (bool tangled = true; tangled;)
    {
        tangled = false;
        const size_t n = points.size();
        for (size_t i = 0; i < n && !tangled; i++)
        {
            for (size_t j = i + 1; j < n && !tangled; j++)
            {
                // The loop runs from where it closes over the corners after i up to j, the rest over the corners after j up to i
                loop.clear();
                rest.clear();
                if (Vector2Distance(points[i], points[j]) >= SAME_POINT)
                {
                    float u = 0.0f;
                    float v = 0.0f;
                    if (j == i + 1 || (i == 0 && j + 1 == n) || !segmentsCross(points[i], points[i + 1], points[j], points[(j + 1) % n])) continue;
                    intersect(points[i], points[i + 1] - points[i], points[j], points[(j + 1) % n] - points[j], u, v);
                    loop.push_back(Vector2Lerp(points[i], points[i + 1], u));
                    rest.push_back(loop.back());
                }
                loop.insert(loop.end(), points.begin() + (long)i + 1, points.begin() + (long)j + 1);
                rest.insert(rest.end(), points.begin() + (long)j + 1, points.end());
                rest.insert(rest.end(), points.begin(), points.begin() + (long)i + 1);
                points = fabsf(signedArea(loop)) > fabsf(signedArea(rest)) ? loop : rest;
                tangled = true;
            }
        }
    }
}

// The crossings of the sides with the line through origin, in order along it. In pairs they bound the parts inside the polygon
static void lineCrossings(std::span<const Vector2> polygon, Vector2 origin, Vector2 direction, std::vector<Crossing> &out)
{
    const Vector2 normal = { -direction.y, direction.x };
    out.clear();
    for (size_t i = 0; i < polygon.size(); i++)
    {
        const Vector2 a = polygon[i];
        const Vector2 b = polygon[(i + 1) % polygon.size()];
        const float sideA = Vector2DotProduct(normal, a - origin);
        const float sideB = Vector2DotProduct(normal, b - origin);
        if ((sideA >= 0.0f) == (sideB >= 0.0f)) continue;

        const Vector2 point = Vector2Lerp(a, b, sideA / (sideA - sideB));
        out.push_back({ i, point, Vector2DotProduct(direction, point - origin), false });
    }
    std::sort(out.begin(), out.end(), [](const Crossing &first, const Crossing &second) { return first.t < second.t; });
}

// Appends where the line crosses the line of a road, both count up to touch past their ends, so that a line can start on a road
static void lineHits(const Polyline &line, const Polyline &road, size_t segment, float touch, std::vector<Hit> &hits)
{
    for (int i = 0; i + 1 < line.count; i++)
    {
        for (int j = 0; j + 1 < road.count; j++)
        {
            float u = 0.0f;
            float v = 0.0f;
            if (!intersect(line.points[i], line.points[i + 1] - line.points[i], road.points[j], road.points[j + 1] - road.points[j], u, v) ||
                u < -touch || u > 1.0f + touch || v < -touch || v > 1.0f + touch) continue;
            hits.push_back({ ((float)i + u) / (float)(line.count - 1), segment, ((float)j + Clamp(v, 0.0f, 1.0f)) / (float)(road.count - 1) });
        }
    }
}

// True when the road runs through the polygon deeper than the clearance, sampled finely so that no straight road slips through
static bool roadCrossesPolygon(const Polyline &line, std::span<const Vector2> polygon, Box box, float clearance)
{
    // A sample is no deeper in the polygon than it is far from any side, and the samples after it no deeper than that plus how far they
    // are from it, so the ones that cannot be deeper than clearance are skipped. The side that was nearest is tried first, along a side
    // of the polygon it is all a sample needs
    if (polygon.size() < 3) return false;

    size_t nearest = 0;
    for (int i = 0; i + 1 < line.count; i++)
    {
        const Vector2 a = line.points[i];
        const Vector2 b = line.points[i + 1];
        if (!overlap(boxOf(std::array<Vector2, 2>{ a, b }), box)) continue;

        const int steps = (int)ceilf(Vector2Distance(a, b) / (0.5f * clearance));
        const float step = steps > 0 ? Vector2Distance(a, b) / (float)steps : 0.0f;
        for (int k = 0; k <= steps; k++)
        {
            const Vector2 p = Vector2Lerp(a, b, steps > 0 ? (float)k / (float)steps : 0.0f);
            if (!contains(box, p)) continue;

            float depth = pointSegmentDistance(p, polygon[nearest], polygon[(nearest + 1) % polygon.size()]);
            if (depth > clearance)
            {
                for (size_t e = 0; e < polygon.size(); e++)
                {
                    const float distance = pointSegmentDistance(p, polygon[e], polygon[(e + 1) % polygon.size()]);
                    if (distance >= depth) continue;

                    depth = distance;
                    nearest = e;
                }
                const bool inside = insidePolygon(polygon, p);
                if (inside && depth > clearance) return true;
                if (!inside) depth = -depth;
            }
            if (step > 0.0f) k += (int)fminf(0.99f * (clearance - depth) / step, (float)steps);
        }
    }
    return false;
}

// True when two parts of a curve far apart along it come closer than clearance: it crosses itself, folds back or bends too tight
static bool curveTouchesItself(const Polyline &line, float clearance)
{
    std::array<float, CURVE_POINTS> arc = {};
    for (int i = 1; i < line.count; i++) arc[i] = arc[i - 1] + Vector2Distance(line.points[i - 1], line.points[i]);
    for (int i = 0; i + 1 < line.count; i++)
    {
        for (int j = i + 2; j + 1 < line.count; j++)
        {
            if (arc[j] - arc[i + 1] >= 2.0f * clearance && segmentsDistance(line.points[i], line.points[i + 1], line.points[j], line.points[j + 1]) < clearance) return true;
        }
    }
    return false;
}

// Whether a piece of one curve is nearly parallel to a piece of the other, closer than gap and beside it for more than minShared
static bool curvesRunAlong(const Polyline &first, const Polyline &second, float gap, float minShared)
{
    // Sine of about 12 degrees, below it two pieces count as parallel
    constexpr float MAX_SINE = 0.21f;

    for (int i = 0; i + 1 < first.count; i++)
    {
        const Vector2 p0 = first.points[i];
        const Vector2 p1 = first.points[i + 1];
        const Vector2 direction = Vector2Normalize(p1 - p0);
        for (int j = 0; j + 1 < second.count; j++)
        {
            const Vector2 q0 = second.points[j];
            const Vector2 q1 = second.points[j + 1];
            if (fabsf(Vector2CrossProduct(direction, Vector2Normalize(q1 - q0))) > MAX_SINE || segmentsDistance(p0, p1, q0, q1) >= gap) continue;

            const float a0 = Vector2DotProduct(p0, direction);
            const float a1 = Vector2DotProduct(p1, direction);
            const float b0 = Vector2DotProduct(q0, direction);
            const float b1 = Vector2DotProduct(q1, direction);
            if (fminf(fmaxf(a0, a1), fmaxf(b0, b1)) - fmaxf(fminf(a0, a1), fminf(b0, b1)) > minShared) return true;
        }
    }
    return false;
}

// Squared distance from the point to the closest point of the line, with that point and its curve parameter
static float closestOnLine(const Polyline &line, Vector2 point, float &t, Vector2 &closest)
{
    float bestSquared = FLT_MAX;
    for (int i = 0; i + 1 < line.count; i++)
    {
        const Vector2 a = line.points[i];
        const Vector2 ab = line.points[i + 1] - a;
        if (Vector2LengthSqr(ab) <= 0.0f) continue;

        const float f = Clamp(Vector2DotProduct(point - a, ab) / Vector2LengthSqr(ab), 0.0f, 1.0f);
        const Vector2 onLine = a + ab * f;
        if (Vector2DistanceSqr(onLine, point) > bestSquared) continue;

        bestSquared = Vector2DistanceSqr(onLine, point);
        t = ((float)i + f) / (float)(line.count - 1);
        closest = onLine;
    }
    return bestSquared;
}

static Interval projectedExtent(std::span<const Vector2> points, Vector2 axis)
{
    Interval extent = { FLT_MAX, -FLT_MAX };
    for (Vector2 p : points) extent = { fminf(extent.low, Vector2DotProduct(p, axis)), fmaxf(extent.high, Vector2DotProduct(p, axis)) };
    return extent;
}

static OrientedBox orientedBox(std::span<const Vector2> points)
{
    OrientedBox best = { { 1.0f, 0.0f }, { 0.0f, 1.0f }, 0.0f, 0.0f };
    float bestArea = FLT_MAX;
    for (size_t i = 0; i < points.size(); i++)
    {
        const Vector2 side = points[(i + 1) % points.size()] - points[i];
        if (Vector2Length(side) < 0.000001f) continue;

        // Both extents in one pass, which stops once the box is not smaller than the best one
        const Vector2 along = Vector2Normalize(side);
        const Vector2 across = { -along.y, along.x };
        Interval alongExtent = { FLT_MAX, -FLT_MAX };
        Interval acrossExtent = { FLT_MAX, -FLT_MAX };
        float area = 0.0f;
        for (size_t k = 0; k < points.size() && area < bestArea; k++)
        {
            const float a = Vector2DotProduct(points[k], along);
            const float b = Vector2DotProduct(points[k], across);
            alongExtent = { std::min(alongExtent.low, a), std::max(alongExtent.high, a) };
            acrossExtent = { std::min(acrossExtent.low, b), std::max(acrossExtent.high, b) };
            area = (alongExtent.high - alongExtent.low) * (acrossExtent.high - acrossExtent.low);
        }
        if (area >= bestArea) continue;

        const float length = alongExtent.high - alongExtent.low;
        const float width = acrossExtent.high - acrossExtent.low;
        bestArea = area;
        best = length >= width ? OrientedBox{ along, across, length, width } : OrientedBox{ across, along, width, length };
    }
    return best;
}

// Corners of a polygon into the buffer, sides that meet at more than 135 degrees count as one side. A smooth outline keeps every point
static std::span<const Vector2> approximateCorners(std::span<const Vector2> p, std::vector<Vector2> &corners)
{
    // Cosine of 135 degrees
    constexpr float COLLINEAR_COS = -0.70710678f;

    const size_t n = p.size();
    corners.clear();
    for (size_t i = 0; i < n; i++)
    {
        if (Vector2DotProduct(Vector2Normalize(p[(i + n - 1) % n] - p[i]), Vector2Normalize(p[(i + 1) % n] - p[i])) >= COLLINEAR_COS) corners.push_back(p[i]);
    }
    if (corners.size() < 3) corners.assign(p.begin(), p.end());
    return corners;
}

// The mean length of the sides between the corners of approximateCorners
static float meanSideLength(std::span<const Vector2> corners)
{
    float sum = 0.0f;
    for (size_t i = 0; i < corners.size(); i++) sum += Vector2Distance(corners[i], corners[(i + 1) % corners.size()]);
    return sum / (float)corners.size();
}

// Shape irregularity of the corners of a counter-clockwise polygon by Equation 1 of Chen et al., zero for a regular polygon
static float shapeIrregularity(std::span<const Vector2> corners, std::vector<float> &angles, std::vector<float> &lengths)
{
    constexpr float ANGLE_WEIGHT = 0.75f;
    constexpr float LENGTH_WEIGHT = 0.25f;

    const size_t n = corners.size();
    angles.resize(n);
    lengths.resize(n);
    float meanAngle = 0.0f;
    float meanLength = 0.0f;
    for (size_t i = 0; i < n; i++)
    {
        // The inner angle is a half turn minus the turn to the left
        const Vector2 in = corners[i] - corners[(i + n - 1) % n];
        const Vector2 out = corners[(i + 1) % n] - corners[i];
        angles[i] = PI - atan2f(Vector2CrossProduct(in, out), Vector2DotProduct(in, out));
        lengths[i] = Vector2Length(out);
        meanAngle += angles[i] / (float)n;
        meanLength += lengths[i] / (float)n;
    }

    float angleVariance = 0.0f;
    float lengthVariance = 0.0f;
    for (size_t i = 0; i < n; i++)
    {
        angleVariance += (angles[i] - meanAngle) * (angles[i] - meanAngle) / (float)n;
        lengthVariance += (lengths[i] - meanLength) * (lengths[i] - meanLength) / (float)n;
    }
    return ANGLE_WEIGHT * angleVariance + LENGTH_WEIGHT * lengthVariance / fmaxf(meanLength * meanLength, 0.000001f);
}

// ---- Street patterns of the parcel types: the organic, raster and radial morphologies that Shariatpour et al. grow with the streets
// of CityEngine (Computational Urban Science 6, 2026), and random streets. They cut a parcel longer than a block into blocks

// A street from a to b that meanders like an organic street of CityEngine, each piece turning by up to maxTurn against the one before,
// inside the polygon and back on the straight line at both ends. The pieces follow a Catmull-Rom spline
static void addMeanderingStreet(std::span<const Vector2> polygon, Vector2 a, Vector2 b, float step, float maxTurn, Random &random, std::vector<Street> &out)
{
    constexpr int MAX_CORNERS = 33;

    std::array<Vector2, MAX_CORNERS> corners = {};
    const int count = std::clamp((int)ceilf(Vector2Distance(a, b) / step), 1, MAX_CORNERS - 1) + 1;
    const float piece = Vector2Distance(a, b) / (float)(count - 1);
    float heading = 0.0f;
    float offset = 0.0f;
    for (int k = 0; k < count; k++)
    {
        // The offset from the straight line fades out at both ends and is dropped close to a side
        const float along = (float)k / (float)(count - 1);
        heading = Clamp(heading + maxTurn * (2.0f * random.Next() - 1.0f), -2.0f * maxTurn, 2.0f * maxTurn);
        offset = Clamp(offset + tanf(heading) * piece, -step, step);
        corners[k] = Vector2Lerp(a, b, along) + leftNormal(a, b) * (offset * sinf(PI * along));
        if (k > 0 && k + 1 < count && (!insidePolygon(polygon, corners[k]) || boundaryDistance(polygon, corners[k]) < 0.25f * step)) corners[k] = Vector2Lerp(a, b, along);
    }
    for (int k = 0; k + 1 < count; k++)
    {
        const Vector2 first = corners[k] + (corners[k + 1] - corners[std::max(k - 1, 0)]) / 6.0f;
        out.push_back({ corners[k], corners[k + 1], { .curved = true, .controls = { first, corners[k + 1] - (corners[std::min(k + 2, count - 1)] - corners[k]) / 6.0f } } });
    }
}

// Parallel streets that cut the polygon into blocks, moved by up to half the jitter of a block, meandering when step is set
static void addParallelStreets(std::span<const Vector2> polygon, Vector2 direction, int blocks, float jitter, float step, float maxTurn, float minLength, Random &random, std::vector<Street> &out)
{
    const Vector2 normal = { -direction.y, direction.x };
    const Interval across = projectedExtent(polygon, normal);
    std::vector<Crossing> crossings;
    for (int i = 1; i < blocks; i++)
    {
        const float fraction = ((float)i + jitter * (random.Next() - 0.5f)) / (float)blocks;
        lineCrossings(polygon, normal * (across.low + fraction * (across.high - across.low)), direction, crossings);
        for (size_t k = 0; k + 1 < crossings.size(); k += 2)
        {
            const Vector2 a = crossings[k].point;
            const Vector2 b = crossings[k + 1].point;
            if (Vector2Distance(a, b) < minLength) continue;
            if (step > 0.0f) addMeanderingStreet(polygon, a, b, step, maxTurn, random, out);
            else out.push_back({ a, b, RoadAttributes() });
        }
    }
}

// The parts of the ring around center that are inside the polygon, cut in arcs of at most 60 degrees that each fit a circle well
static void addRing(std::span<const Vector2> polygon, Vector2 center, float radius, float minLength, std::vector<Street> &out)
{
    constexpr float MAX_ARC = PI / 3.0f;

    // The angles where the sides cross the circle, the roots of |f + s d| = radius along each side
    std::vector<float> angles;
    for (size_t i = 0; i < polygon.size(); i++)
    {
        const Vector2 f = polygon[i] - center;
        const Vector2 d = polygon[(i + 1) % polygon.size()] - polygon[i];
        const float a = Vector2LengthSqr(d);
        const float b = 2.0f * Vector2DotProduct(f, d);
        const float discriminant = b * b - 4.0f * a * (Vector2LengthSqr(f) - radius * radius);
        if (a <= 0.0f || discriminant < 0.0f) continue;

        for (float s : { (-b - sqrtf(discriminant)) / (2.0f * a), (-b + sqrtf(discriminant)) / (2.0f * a) })
        {
            const Vector2 p = f + d * s;
            if (s >= 0.0f && s < 1.0f) angles.push_back(atan2f(p.y, p.x));
        }
    }
    std::sort(angles.begin(), angles.end());

    // Between two crossings the circle is inside or outside all along, its middle tells. Without crossings it is one full turn
    if (angles.empty()) angles.push_back(0.0f);
    for (size_t i = 0; i < angles.size(); i++)
    {
        const float from = angles[i];
        const float to = i + 1 < angles.size() ? angles[i + 1] : angles[0] + 2.0f * PI;
        const float middle = 0.5f * (from + to);
        if (to - from <= 0.0001f || !insidePolygon(polygon, center + Vector2{ cosf(middle), sinf(middle) } *radius)) continue;

        // The control point of an arc is where the tangents at its ends meet
        const int pieces = (int)ceilf((to - from) / MAX_ARC);
        for (int k = 0; k < pieces; k++)
        {
            const float start = from + (to - from) * (float)k / (float)pieces;
            const float end = from + (to - from) * (float)(k + 1) / (float)pieces;
            const Vector2 a = center + Vector2{ cosf(start), sinf(start) } *radius;
            const Vector2 b = center + Vector2{ cosf(end), sinf(end) } *radius;
            const Vector2 control = center + Vector2{ cosf(0.5f * (start + end)), sinf(0.5f * (start + end)) } *(radius / cosf(0.5f * (end - start)));
            if (Vector2Distance(a, b) >= 0.25f * minLength) out.push_back({ a, b, bentRoad(a, control, b) });
        }
    }
}

// Rings around the middle half a block apart, and streets out of the middle that double further out to keep blocks a block long
static void addRadialStreets(std::span<const Vector2> polygon, Vector2 center, Vector2 axis, const ParcelAttributes &attributes, std::vector<Street> &out)
{
    constexpr int FIRST_STREETS = 4;

    const float spacing = 0.5f * attributes.settings.maxBlockSize;
    const float minLength = 0.5f * attributes.settings.minBlockSize;
    float reach = 0.0f;
    for (Vector2 p : polygon) reach = fmaxf(reach, Vector2Distance(center, p));

    std::vector<Crossing> crossings;
    int streets = FIRST_STREETS;
    for (float inner = 0.5f * spacing; inner < reach; inner += spacing)
    {
        // The ring unless it would run close to the farthest corner, then the streets out to the next ring or to the edge
        if (inner + 0.25f * spacing < reach) addRing(polygon, center, inner, minLength, out);
        const float outer = inner + 1.25f * spacing < reach ? inner + spacing : FLT_MAX;
        while (2.0f * PI * fminf(outer, reach) / (float)streets > attributes.settings.maxBlockSize) streets *= 2;
        for (int j = 0; j < streets; j++)
        {
            const Vector2 direction = Vector2Rotate(axis, 2.0f * PI * (float)j / (float)streets);
            lineCrossings(polygon, center, direction, crossings);
            for (size_t k = 0; k + 1 < crossings.size(); k += 2)
            {
                const float from = fmaxf(crossings[k].t, inner);
                const float to = fminf(crossings[k + 1].t, outer);
                if (to - from >= minLength) out.push_back({ center + direction * from, center + direction * to, RoadAttributes() });
            }
        }
    }
}

// Blocks across the direction so that no block is wider than the block size
static int blockCount(std::span<const Vector2> polygon, Vector2 direction, float blockSize)
{
    const Interval across = projectedExtent(polygon, { -direction.y, direction.x });
    return std::max(1, (int)ceilf((across.high - across.low) / blockSize));
}

// Streets for a parcel that is longer than a block, radial streets go around the middle in its attributes
static void generateStreetPattern(std::span<const Vector2> outline, const ParcelAttributes &attributes, unsigned seed, std::vector<Street> &out)
{
    // Organic streets meander in pieces of a fifth of a block that turn by up to 20 degrees, their spacing and angles change this much
    constexpr float ORGANIC_STEP = 0.2f;
    constexpr float ORGANIC_TURN = 0.35f;
    constexpr float ORGANIC_JITTER = 0.7f;
    constexpr float ORGANIC_ANGLE = 0.6f;

    out.clear();
    const OrientedBox box = orientedBox(outline);
    if (box.length <= attributes.settings.maxBlockSize) return;

    Random random = { seed * 2246822519u | 1u };
    const float longSide = attributes.settings.maxBlockSize;
    const float shortSide = 0.5f * attributes.settings.maxBlockSize;
    const float minLength = 0.5f * attributes.settings.minBlockSize;
    switch (attributes.style)
    {
        case SubdivisionStyle::Grid:
        {
            // Raster streets across the parcel a block apart and along it half a block apart
            addParallelStreets(outline, box.axisShort, blockCount(outline, box.axisShort, longSide), 0.0f, 0.0f, 0.0f, minLength, random, out);
            addParallelStreets(outline, box.axisLong, blockCount(outline, box.axisLong, shortSide), 0.0f, 0.0f, 0.0f, minLength, random, out);
            break;
        }
        case SubdivisionStyle::Organic:
        {
            // Two families of meandering streets that cross at changing angles, with uneven spacing
            const Vector2 first = Vector2Rotate(box.axisLong, ORGANIC_ANGLE * (random.Next() - 0.5f));
            const Vector2 second = Vector2Rotate(first, 0.5f * PI + ORGANIC_ANGLE * (random.Next() - 0.5f));
            const float step = ORGANIC_STEP * attributes.settings.maxBlockSize;
            addParallelStreets(outline, first, blockCount(outline, first, shortSide), ORGANIC_JITTER, step, ORGANIC_TURN, minLength, random, out);
            addParallelStreets(outline, second, blockCount(outline, second, longSide), ORGANIC_JITTER, step, ORGANIC_TURN, minLength, random, out);
            break;
        }
        case SubdivisionStyle::Radial:
        {
            addRadialStreets(outline, attributes.center, box.axisLong, attributes, out);
            break;
        }
        case SubdivisionStyle::Random:
        {
            // Straight streets at any angle somewhere in the middle part, as many as a grid would need
            const int streets = blockCount(outline, box.axisShort, longSide) + blockCount(outline, box.axisLong, shortSide);
            for (int k = 0; k < streets; k++)
            {
                const float angle = PI * random.Next();
                addParallelStreets(outline, { cosf(angle), sinf(angle) }, 2, 1.2f, 0.0f, 0.0f, minLength, random, out);
            }
            break;
        }
    }
}

// ---- Lots

// Parallel lines spread from low to high over the width of the polygon across the direction, while there is room for them
static void addParallelLines(std::span<const Vector2> points, Vector2 direction, int lineCount, float low, float high, std::span<Line> lines, int &count)
{
    const Vector2 normal = { -direction.y, direction.x };
    const Interval across = projectedExtent(points, normal);
    for (int k = 0; k < lineCount && count < (int)lines.size(); k++)
    {
        const float fraction = lineCount > 1 ? low + (high - low) * (float)k / (float)(lineCount - 1) : low;
        lines[count++] = { normal * (across.low + fraction * (across.high - across.low)), direction };
    }
}

// The length of the sides of the cell that run along a street
static float frontage(const Cell &cell)
{
    float length = 0.0f;
    for (size_t i = 0; i < cell.points.size(); i++)
    {
        if (cell.street[i]) length += Vector2Distance(cell.points[i], cell.points[(i + 1) % cell.points.size()]);
    }
    return length;
}

// Whether a crossing is on a street, on a corner either side may be one
static bool onStreet(const Cell &cell, const Crossing &c)
{
    return cell.street[c.side] || (c.corner && cell.street[(c.side + cell.street.size() - 1) % cell.street.size()]);
}

// A grid over the boxes of the roads, to find the roads near a lot
static BoxGrid roadGrid(const RoadGraph &graph)
{
    std::vector<Box> boxes;
    for (const RoadSegment &segment : graph.segments) boxes.push_back(segment.box);
    BoxGrid grid;
    grid.Build(boxes);
    return grid;
}

// The parcel that a road lies inside, like a dead end, its middle is in it and not on a side. SIZE_MAX for a road between two parcels
static size_t parcelAround(const RoadGraph &graph, const ParcelMesh &mesh, size_t segment)
{
    // How far from the sides the middle of a road inside a parcel is at least, the middle of a road along a side is on it
    constexpr float ON_SIDE = 0.01f;

    size_t at = 0;
    const Vector2 middle = lineMiddle(graph.segments[segment].line);
    return mesh.Find(middle, at) && boundaryDistance(mesh.Outline(at), middle) > ON_SIDE ? at : SIZE_MAX;
}

// The lines of the roads that lie inside the parcel
static void roadsInside(const RoadGraph &graph, const ParcelMesh &mesh, size_t parcel, std::vector<Polyline> &roads)
{
    roads.clear();
    for (size_t s = 0; s < graph.segments.size(); s++)
    {
        if (contains(mesh.shapes[parcel].box, lineMiddle(graph.segments[s].line)) && parcelAround(graph, mesh, s) == parcel) roads.push_back(graph.segments[s].line);
    }
}

// The stretches of a curve that do not run along a road closer than distance, as pairs of curve parameters that end on the roads
static void offRoads(const RoadGraph &graph, const Bezier &curve, float distance, std::vector<float> &stretches)
{
    // Sine of about 12 degrees, below it a road counts as running along
    constexpr float MAX_SINE = 0.21f;

    // The roads near the curve, then samples about distance apart
    const Box box = inflate(boxOf(curve.points), distance);
    std::vector<size_t> near;
    for (size_t s = 0; s < graph.segments.size(); s++)
    {
        if (overlap(box, graph.segments[s].box)) near.push_back(s);
    }
    const float length = Vector2Distance(curve.points[0], curve.points[1]) + Vector2Distance(curve.points[1], curve.points[2]) + Vector2Distance(curve.points[2], curve.points[3]);
    const float step = 1.0f / (float)std::max(2, (int)ceilf(length / distance));
    stretches.clear();
    for (float t = 0.0f; t < 1.0f + 0.5f * step; t += step)
    {
        const Vector2 p = bezierPoint(curve, fminf(t, 1.0f));
        const Vector2 tangent = Vector2Normalize(bezierPoint(curve, fminf(t + 0.5f * step, 1.0f)) - bezierPoint(curve, fmaxf(t - 0.5f * step, 0.0f)));
        bool along = false;
        for (size_t k = 0; k < near.size() && !along; k++)
        {
            const Polyline &line = graph.segments[near[k]].line;
            float u = 0.0f;
            Vector2 closest = p;
            if (closestOnLine(line, p, u, closest) >= distance * distance) continue;

            // A road that only ends next to the curve, like one it goes on from, does not run along it
            const int piece = std::min((int)(u * (float)(line.count - 1)), line.count - 2);
            along = u > 0.0f && u < 1.0f && fabsf(Vector2CrossProduct(tangent, Vector2Normalize(line.points[piece + 1] - line.points[piece]))) < MAX_SINE;
        }
        if (along == (stretches.size() % 2 == 1)) stretches.push_back(along ? fminf(t, 1.0f) : fmaxf(t - step, 0.0f));
    }
    if (stretches.size() % 2 == 1) stretches.push_back(1.0f);
}

// Whether a road runs along a side of the lot and none runs through it, and every corner is in the parcel of the lot or on its side, not
// in a parcel inside it. Every lot needs all of it
static bool onRoad(const RoadGraph &graph, const BoxGrid &roads, const ParcelMesh &mesh, size_t parcel, std::span<const Vector2> lot)
{
    // How deep a road has to run into a lot or a corner out of its parcel, and how far a road along a side can be, roads next to lots are
    // often a little off
    constexpr float CLEARANCE = 0.6f;
    constexpr float ON_ROAD = 0.35f;

    const Box box = boxOf(lot);
    const GridRange range = roads.Range(box);
    for (int y = range.y0; y <= range.y1; y++)
    {
        for (int x = range.x0; x <= range.x1; x++)
        {
            for (size_t s : roads.At(x, y))
            {
                // A road in more cells of the lot is tested in the first of them
                const GridRange cells = roads.Range(graph.segments[s].box);
                if (x != std::max(cells.x0, range.x0) || y != std::max(cells.y0, range.y0)) continue;
                if (overlap(box, graph.segments[s].box) && roadCrossesPolygon(graph.segments[s].line, lot, box, CLEARANCE)) return false;
            }
        }
    }
    // Most corners of a lot are on a side of its parcel, the side found last is tried first. A corner off the sides has to be in the parcel
    const std::span<const Vector2> outline = mesh.Outline(parcel);
    size_t side = 0;
    for (Vector2 corner : lot)
    {
        bool onSide = false;
        for (size_t k = 0; k < outline.size() && !onSide; k++)
        {
            const size_t i = (side + k) % outline.size();
            onSide = pointSegmentDistance(corner, outline[i], outline[(i + 1) % outline.size()]) <= CLEARANCE;
            if (onSide) side = i;
        }
        size_t at = 0;
        if (!onSide && !(mesh.Find(corner, at) && at == parcel)) return false;
    }
    for (size_t i = 0; i < lot.size(); i++)
    {
        const Vector2 middle = Vector2Lerp(lot[i], lot[(i + 1) % lot.size()], 0.5f);
        for (size_t s : roads.At(middle))
        {
            float t = 0.0f;
            Vector2 closest = middle;
            if (closestOnLine(graph.segments[s].line, middle, t, closest) < ON_ROAD * ON_ROAD) return true;
        }
    }
    return false;
}

// ---- Road drawing

// The point at a distance along a line with the arc lengths of its samples, the piece it is on only moves forward
static Vector2 pointAlong(const Polyline &line, const std::array<float, CURVE_POINTS> &arc, float distance, int &piece)
{
    while (piece + 2 < line.count && arc[piece + 1] < distance) piece++;
    const float length = arc[piece + 1] - arc[piece];
    return Vector2Lerp(line.points[piece], line.points[piece + 1], length > 0.0f ? Clamp((distance - arc[piece]) / length, 0.0f, 1.0f) : 0.0f);
}

// A dashed line along a road that starts startSkip after its first end and stops endSkip before its other end
static void drawDashes(const Polyline &line, float startSkip, float endSkip, float dash, float gap, float thickness, Color color)
{
    std::array<float, CURVE_POINTS> arc = {};
    for (int i = 1; i < line.count; i++) arc[i] = arc[i - 1] + Vector2Distance(line.points[i - 1], line.points[i]);

    int piece = 0;
    const float end = arc[line.count - 1] - endSkip;
    for (float start = startSkip; start < end; start += dash + gap)
    {
        const Vector2 a = pointAlong(line, arc, start, piece);
        DrawLineEx(a, pointAlong(line, arc, fminf(start + dash, end), piece), thickness, color);
    }
}

// ---- Lots

float Random::Next()
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return (float)(state & 0xFFFFFF) / 16777216.0f;
}

void Cell::Add(Vector2 point, char isStreet)
{
    points.push_back(point);
    street.push_back(isStreet);
}

LotSubdivider::LotSubdivider(const ParcelAttributes &parcelAttributes, unsigned seed) : attributes(parcelAttributes), random{ seed * 2654435761u | 1u }
{}

void LotSubdivider::Run(std::span<const Vector2> outline, std::span<const Polyline> roads, std::vector<Vector2> &lotPoints, std::vector<size_t> &lotSizes, std::vector<Street> &streets)
{
    constexpr size_t MAX_LOTS = 65536;
    constexpr float STRAIGHT = 0.05f;

    // Corners that the line past them misses by less than STRAIGHT, like most samples of a gentle curve, only make it slower
    Cell cell;
    for (size_t i = 0, from = 0; i < outline.size(); i++)
    {
        bool straight = i > 0;
        for (size_t k = from + 1; k <= i && straight; k++) straight = pointSegmentDistance(outline[k], outline[from], outline[(i + 1) % outline.size()]) <= STRAIGHT;
        if (straight) continue;

        cell.points.push_back(outline[i]);
        from = i;
    }
    while (cell.points.size() > 1 && Vector2Distance(cell.points.front(), cell.points.back()) < SAME_POINT) cell.points.pop_back();

    // Leaving out corners can make the sides of a thin spike cross, and roads can cross without an intersection
    if (!polygonIsSimple(cell.points)) untangle(cell.points);
    if (!polygonIsSimple(cell.points)) return;
    if (signedArea(cell.points) < 0.0f) std::reverse(cell.points.begin(), cell.points.end());
    cell.street.assign(cell.points.size(), 1);

    // Roads that end in the block become slits in it, a road that goes on from another one after that one
    for (size_t pass = 0; pass < roads.size(); pass++)
    {
        for (const Polyline &road : roads) AddSlit(cell, road);
    }

    // Depth first, a cell that is small enough or cannot be split is a lot. The second part of a split waits on a stack that keeps the
    // corners and the street flags of all waiting parts one after another
    std::vector<Vector2> waitingPoints;
    std::vector<char> waitingStreets;
    std::vector<size_t> waitingStarts;
    std::array<Cell, 2> parts;
    size_t lots = 0;
    for (;;)
    {
        const float width = orientedBox(cell.points).width;
        const bool large = signedArea(cell.points) > attributes.settings.maxParcelArea || width > attributes.settings.maxParcelWidth;
        if (lots + waitingStarts.size() < MAX_LOTS && large && Split(cell, parts, streets))
        {
            waitingStarts.push_back(waitingPoints.size());
            waitingPoints.insert(waitingPoints.end(), parts[1].points.begin(), parts[1].points.end());
            waitingStreets.insert(waitingStreets.end(), parts[1].street.begin(), parts[1].street.end());
            std::swap(cell, parts[0]);
            continue;
        }

        // A cell narrower than a fifth of the narrowest lot is a sliver between roads that the roads hide
        if (width >= 0.2f * attributes.settings.minParcelWidth)
        {
            lotPoints.insert(lotPoints.end(), cell.points.begin(), cell.points.end());
            lotSizes.push_back(cell.points.size());
            lots++;
        }
        if (waitingStarts.empty()) return;

        const size_t start = waitingStarts.back();
        waitingStarts.pop_back();
        cell.points.assign(waitingPoints.begin() + (long)start, waitingPoints.end());
        cell.street.assign(waitingStreets.begin() + (long)start, waitingStreets.end());
        waitingPoints.resize(start);
        waitingStreets.resize(start);
    }
}

int LotSubdivider::CollectLines(std::span<const Vector2> points, const OrientedBox &box, std::array<Line, MAX_LINES> &lines)
{
    constexpr float TURN = 0.26f;

    int count = 0;
    switch (attributes.style)
    {
        case SubdivisionStyle::Grid:
        {
            // Lines across and along the cell, a regular grid of lots
            addParallelLines(points, box.axisShort, 9, 0.25f, 0.75f, lines, count);
            addParallelLines(points, box.axisLong, 9, 0.25f, 0.75f, lines, count);
            break;
        }
        case SubdivisionStyle::Organic:
        {
            // Slightly turned lines, the lots adapt to the shape
            for (int turn = -1; turn <= 1; turn++)
            {
                addParallelLines(points, Vector2Rotate(box.axisShort, (float)turn * TURN), 3, 0.35f, 0.65f, lines, count);
                addParallelLines(points, Vector2Rotate(box.axisLong, (float)turn * TURN), 3, 0.35f, 0.65f, lines, count);
            }
            break;
        }
        case SubdivisionStyle::Radial:
        {
            // Lines out of the middle over the angle of the cell and lines around it make wedges, a cell around the middle is cut through it
            const Vector2 outward = Vector2Normalize(interiorPoint(points) - attributes.center);
            if (insidePolygon(points, attributes.center) || Vector2Length(outward) < 0.5f)
            {
                for (int k = 0; k < 9; k++) lines[count++] = { attributes.center, Vector2Rotate(box.axisLong, PI * (float)k / 9.0f) };
                break;
            }
            Interval angles = { 0.0f, 0.0f };
            for (Vector2 p : points) angles = { fminf(angles.low, Vector2Angle(outward, p - attributes.center)), fmaxf(angles.high, Vector2Angle(outward, p - attributes.center)) };
            for (int k = 0; k < 9; k++) lines[count++] = { attributes.center, Vector2Rotate(outward, angles.low + (angles.high - angles.low) * (0.25f + 0.0625f * (float)k)) };
            addParallelLines(points, { -outward.y, outward.x }, 9, 0.25f, 0.75f, lines, count);
            break;
        }
        case SubdivisionStyle::Random:
        {
            for (int k = 0; k < MAX_LINES; k++)
            {
                const float angle = PI * random.Next();
                addParallelLines(points, { cosf(angle), sinf(angle) }, 1, 0.3f + 0.4f * random.Next(), 0.0f, lines, count);
            }
            break;
        }
    }
    return count;
}

bool LotSubdivider::Split(const Cell &cell, std::array<Cell, 2> &parts, std::vector<Street> &streets)
{
    // How much the irregularity setting can change the quality of a candidate, and how close to a corner a chord moves onto it
    constexpr float NOISE_SCALE = 0.5f;
    constexpr float SNAP_DISTANCE = 0.1f;

    std::array<Line, MAX_LINES> lines;
    const OrientedBox box = orientedBox(cell.points);
    const int lineCount = CollectLines(cell.points, box, lines);
    const bool wide = box.width > 0.5f * attributes.settings.maxBlockSize;
    const size_t n = cell.points.size();
    best.quality = -FLT_MAX;
    for (int l = 0; l < lineCount; l++)
    {
        // A crossing close to a corner moves onto it, so splits leave no slivers
        lineCrossings(cell.points, lines[l].origin, lines[l].direction, crossings);
        for (Crossing &c : crossings)
        {
            const size_t next = (c.side + 1) % n;
            if (Vector2Distance(c.point, cell.points[c.side]) < SNAP_DISTANCE) c = { c.side, cell.points[c.side], c.t, true };
            else if (Vector2Distance(c.point, cell.points[next]) < SNAP_DISTANCE) c = { next, cell.points[next], c.t, true };
        }

        for (size_t k = 0; k + 1 < crossings.size(); k += 2)
        {
            if (!Evaluate(cell, crossings[k], crossings[k + 1], wide, candidate)) continue;

            candidate.quality += attributes.settings.splitIrregularity * NOISE_SCALE * (2.0f * random.Next() - 1.0f);
            if (candidate.quality > best.quality) std::swap(best, candidate);
        }
    }
    if (best.quality == -FLT_MAX) return false;

    // A bent chord cuts the parts again along every sample of its road, so that they end where the road will run
    if (best.chord.road.curved)
    {
        const Polyline line = roadLine(best.chord.a, best.chord.road, best.chord.b);
        Cut(cell, best.ends[0], best.ends[1], std::span<const Vector2>(line.points).subspan(1, CURVE_POINTS - 2), best.street ? 1 : 0, best.parts);
    }
    if (best.street) streets.push_back(best.chord);
    std::swap(parts, best.parts);
    return true;
}

bool LotSubdivider::Evaluate(const Cell &cell, Crossing a, Crossing b, bool wide, Candidate &split)
{
    constexpr float SIZE_WEIGHT = 0.3f;
    constexpr float REGULARITY_WEIGHT = 0.5f;
    constexpr float ACCESS_WEIGHT = 0.2f;

    // A part counts as reached when its street is at least half its mean side long
    constexpr float ACCESS_RATIO = 0.5f;

    if (a.side == b.side) return false;
    if (a.side > b.side) std::swap(a, b);

    std::array<Cell, 2> &parts = split.parts;
    split.ends = { a, b };
    split.chord = { a.point, b.point, RoadAttributes() };
    Cut(cell, a, b, {}, 0, parts);
    const float frontages[2] = { frontage(parts[0]), frontage(parts[1]) };
    const ParcelSettings &settings = attributes.settings;
    split.street = wide || frontages[0] < settings.minParcelWidth || frontages[1] < settings.minParcelWidth;
    if (split.street && ((!onStreet(cell, a) && !onStreet(cell, b)) || Vector2Distance(a.point, b.point) < settings.minParcelWidth)) return false;

    // Organic chords bend, the parts follow the curve. Whether the parts are reached is measured with the straight chord
    std::array<Vector2, BEND_POINTS + 2> bend = {};
    const bool bent = attributes.style == SubdivisionStyle::Organic && Bend(cell.points, a.point, b.point, bend, split.chord.road);
    float sides[2] = {};
    for (int k = 0; k < 2 && bent; k++) sides[k] = meanSideLength(approximateCorners(parts[k].points, corners));
    if (bent || split.street) Cut(cell, a, b, bent ? std::span<const Vector2>(bend).subspan(1, BEND_POINTS) : std::span<const Vector2>(), split.street ? 1 : 0, parts);

    // The parts are measured only when the split is not refused, a straight chord leaves their corners as they were
    float area[2] = {};
    float irregularity[2] = {};
    for (int k = 0; k < 2; k++)
    {
        area[k] = signedArea(parts[k].points);
        if (parts[k].points.size() < 3 || area[k] < settings.minParcelArea || orientedBox(parts[k].points).width < settings.minParcelWidth) return false;
    }
    for (int k = 0; k < 2; k++)
    {
        approximateCorners(parts[k].points, corners);
        if (!bent) sides[k] = meanSideLength(corners);
        irregularity[k] = shapeIrregularity(corners, cornerAngles, sideLengths);
    }
    const bool reached = frontages[0] >= ACCESS_RATIO * sides[0] && frontages[1] >= ACCESS_RATIO * sides[1];
    split.quality = SIZE_WEIGHT * fminf(area[0], area[1]) / fmaxf(area[0], area[1]) + REGULARITY_WEIGHT / (1.0f + irregularity[0] + irregularity[1]) +
        ACCESS_WEIGHT * (reached ? 2.0f : 1.0f);
    return true;
}

void LotSubdivider::AddSlit(Cell &cell, const Polyline &road)
{
    // How far from a side a road end is still on it, and half the width of a slit
    constexpr float ON_SIDE = 0.1f;
    constexpr float HALF_WIDTH = 0.02f;

    // The road has to lie in the cell with one end on a side and the other one off the sides farther than a slit is wide, the slit runs
    // from the end on a side
    std::array<Vector2, CURVE_POINTS> line = {};
    const size_t tip = (size_t)road.count - 1;
    const size_t n = cell.points.size();
    bool inside = true;
    for (size_t i = 0; i <= tip && inside; i++) inside = insidePolygon(cell.points, road.points[i]) || boundaryDistance(cell.points, road.points[i]) <= ON_SIDE;
    const float ends[2] = { boundaryDistance(cell.points, road.points[0]), boundaryDistance(cell.points, road.points[tip]) };
    if (!inside || fminf(ends[0], ends[1]) > ON_SIDE || fmaxf(ends[0], ends[1]) <= 2.0f * HALF_WIDTH) return;
    for (size_t i = 0; i <= tip; i++) line[i] = road.points[ends[0] < ends[1] ? i : tip - i];

    // The mouth is on the side nearest the end, or the side after it at a corner. Going in left of the road and back right keeps the winding
    for (int attempt = 0; attempt < 2; attempt++)
    {
        size_t side = 0;
        for (size_t i = 1; i < n; i++)
        {
            if (pointSegmentDistance(line[0], cell.points[i], cell.points[(i + 1) % n]) < pointSegmentDistance(line[0], cell.points[side], cell.points[(side + 1) % n])) side = i;
        }
        side = (side + (size_t)attempt) % n;
        const Vector2 a = cell.points[side];
        const Vector2 direction = Vector2Normalize(cell.points[(side + 1) % n] - a);
        const float length = Vector2Distance(a, cell.points[(side + 1) % n]);

        // On a short side, like the end of a slit where another road goes on, the mouth narrows
        const float mouth = fminf(HALF_WIDTH, 0.25f * length);
        const float at = Clamp(Vector2DotProduct(line[0] - a, direction), 2.0f * mouth, length - 2.0f * mouth);
        Cell slit;
        slit.Add(a + direction * (at - mouth), 1);
        for (size_t k = 1; k <= 2 * tip + 1; k++)
        {
            const size_t i = k <= tip ? k : k == tip + 1 ? tip : 2 * tip + 2 - k;
            const Vector2 tangent = Vector2Normalize(line[std::min(i + 1, tip)] - line[i - 1]);
            slit.Add(line[i] + (k <= tip ? Vector2{ -tangent.y, tangent.x } : k == tip + 1 ? tangent : Vector2{ tangent.y, -tangent.x }) * HALF_WIDTH, 1);
        }
        slit.Add(a + direction * (at + mouth), cell.street[side]);

        Cell before = cell;
        cell.points.insert(cell.points.begin() + (long)side + 1, slit.points.begin(), slit.points.end());
        cell.street.insert(cell.street.begin() + (long)side + 1, slit.street.begin(), slit.street.end());
        if (polygonIsSimple(cell.points)) return;

        std::swap(cell, before);
    }
}

void LotSubdivider::Cut(const Cell &cell, const Crossing &a, const Crossing &b, std::span<const Vector2> bend, char chordStreet, std::array<Cell, 2> &parts)
{
    // The first part runs from a along the sides to b and back along the chord, the second from b along the other sides to a
    const size_t n = cell.points.size();
    for (Cell &part : parts)
    {
        part.points.clear();
        part.street.clear();
    }
    parts[0].Add(a.point, cell.street[a.side]);
    for (size_t k = a.side + 1; k <= b.side; k++) parts[0].Add(cell.points[k], cell.street[k]);
    parts[0].Add(b.point, chordStreet);
    for (size_t k = bend.size(); k-- > 0;) parts[0].Add(bend[k], chordStreet);
    parts[1].Add(b.point, cell.street[b.side]);
    for (size_t k = b.side + 1; k <= n + a.side; k++) parts[1].Add(cell.points[k % n], cell.street[k % n]);
    parts[1].Add(a.point, chordStreet);
    for (Vector2 p : bend) parts[1].Add(p, chordStreet);

    // A chord that ends on a corner repeats it, the side that leaves the repeated corner is the one that stays
    for (Cell &part : parts)
    {
        for (size_t i = 0; i < part.points.size() && part.points.size() > 2;)
        {
            if (Vector2Distance(part.points[i], part.points[(i + 1) % part.points.size()]) >= SAME_POINT) i++;
            else
            {
                part.points.erase(part.points.begin() + (long)i);
                part.street.erase(part.street.begin() + (long)i);
            }
        }
    }
}

bool LotSubdivider::Bend(std::span<const Vector2> polygon, Vector2 a, Vector2 b, std::array<Vector2, BEND_POINTS + 2> &line, RoadAttributes &road)
{
    // The most the chord bends in parts of its length, the length below which it stays straight, the room kept from the sides
    constexpr float MAX_BEND = 0.2f;
    constexpr float MIN_LENGTH = 12.0f;
    constexpr float CLEARANCE = 0.3f;

    if (Vector2Distance(a, b) < MIN_LENGTH) return false;

    const float sign = random.Next() < 0.5f ? 1.0f : -1.0f;
    const float amount = 2.0f * MAX_BEND * (0.3f + 0.7f * random.Next());
    const Vector2 side = Vector2{ a.y - b.y, b.x - a.x } *(sign * amount);
    line[0] = a;
    line[BEND_POINTS + 1] = b;
    for (float flip : { 1.0f, -1.0f })
    {
        road = bentRoad(a, Vector2Lerp(a, b, 0.5f) + side * flip, b);
        const Bezier curve = roadBezier(a, road, b);
        bool fits = true;
        for (int k = 1; k <= BEND_POINTS && fits; k++)
        {
            line[k] = bezierPoint(curve, (float)k / (float)(BEND_POINTS + 1));
            fits = insidePolygon(polygon, line[k]) && boundaryDistance(polygon, line[k]) > CLEARANCE;
        }
        if (fits && !linesCross(line, polygon, true)) return true;
    }
    road = RoadAttributes();
    return false;
}

// ---- Road graph

RoadAnchor RoadGraph::Snap(Vector2 point, float radius) const
{
    RoadAnchor anchor = { AnchorKind::Free, 0, 0, 0.0f, point };
    if (FindNode(point, radius, anchor.node)) return { AnchorKind::Node, anchor.node, 0, 0.0f, nodes[anchor.node] };

    float bestSquared = radius * radius;
    for (size_t s = 0; s < segments.size(); s++)
    {
        float t = 0.0f;
        Vector2 closest = point;
        if (!contains(inflate(segments[s].box, radius), point) || closestOnLine(segments[s].line, point, t, closest) > bestSquared) continue;

        bestSquared = Vector2DistanceSqr(closest, point);
        anchor = { AnchorKind::Road, 0, s, t, closest };
    }
    return anchor;
}

bool RoadGraph::FindNode(Vector2 point, float radius, size_t &node) const
{
    bool found = false;
    float bestSquared = radius * radius;
    for (size_t n = 0; n < nodes.size(); n++)
    {
        if (Vector2DistanceSqr(nodes[n], point) > bestSquared) continue;

        bestSquared = Vector2DistanceSqr(nodes[n], point);
        node = n;
        found = true;
    }
    return found;
}

bool RoadGraph::FindSegment(Vector2 point, float radius, size_t &index) const
{
    bool found = false;
    float bestSquared = FLT_MAX;
    for (size_t s = 0; s < segments.size(); s++)
    {
        const float reach = fmaxf(radius, segments[s].road.width * 0.5f);
        float t = 0.0f;
        Vector2 closest = point;
        const float distanceSquared = contains(inflate(segments[s].box, reach), point) ? closestOnLine(segments[s].line, point, t, closest) : FLT_MAX;
        if (distanceSquared > reach * reach || distanceSquared >= bestSquared) continue;

        bestSquared = distanceSquared;
        index = s;
        found = true;
    }
    return found;
}

Box RoadGraph::Bounds() const
{
    return nodes.empty() ? Box{ { 0.0f, 0.0f }, { 160.0f, 100.0f } } : boxOf(nodes);
}

bool RoadGraph::CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road) const
{
    return Vector2Distance(a.position, b.position) >= MIN_ROAD_LENGTH && !curveTouchesItself(roadLine(a.position, road, b.position), road.width);
}

bool RoadGraph::RunsAlongRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road) const
{
    const Polyline line = roadLine(a.position, road, b.position);
    const Box box = boxOf(samples(line));
    for (const RoadSegment &segment : segments)
    {
        const float gap = road.width + segment.road.width;
        if (overlap(inflate(box, gap), segment.box) && curvesRunAlong(line, segment.line, gap, 0.5f * gap)) return true;
    }
    return false;
}

bool RoadGraph::AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road)
{
    // Touching the end of a road counts, so a road can end exactly on another road
    constexpr float TOUCH = 0.0001f;

    if (!CanAddRoad(a, b, road)) return false;

    const Polyline line = roadLine(a.position, road, b.position);
    const Box box = boxOf(samples(line));
    std::vector<Hit> hits;
    for (size_t s = 0; s < segments.size(); s++)
    {
        if (overlap(box, segments[s].box)) lineHits(line, segments[s].line, s, TOUCH, hits);
    }

    // Every road is split from its far end on, so the first half keeps its index, a crossing near an end of the road is that end
    std::sort(hits.begin(), hits.end(), [](const Hit &first, const Hit &second) { return first.segment != second.segment ? first.segment < second.segment : first.t > second.t; });
    std::vector<std::pair<float, size_t>> path;
    float scale = 1.0f;
    for (size_t k = 0; k < hits.size(); k++)
    {
        if (k == 0 || hits[k].segment != hits[k - 1].segment) scale = 1.0f;

        const RoadSegment &crossed = segments[hits[k].segment];
        const Vector2 point = bezierPoint(roadBezier(nodes[crossed.from], crossed.road, nodes[crossed.to]), hits[k].t / scale);
        size_t node = Vector2Distance(point, nodes[crossed.from]) < Vector2Distance(point, nodes[crossed.to]) ? crossed.from : crossed.to;
        if (Vector2Distance(point, nodes[node]) >= MERGE_DISTANCE)
        {
            node = SplitSegment(hits[k].segment, hits[k].t / scale);
            scale = hits[k].t;
        }
        path.push_back({ hits[k].along, node });
    }

    // The ends of the new road, then the intersections along it in order, one that comes twice in a row is kept once
    path.push_back({ 0.0f, NodeAt(a.position) });
    path.push_back({ 1.0f, NodeAt(b.position) });
    std::sort(path.begin(), path.end());
    path.erase(std::unique(path.begin(), path.end(), [](const std::pair<float, size_t> &first, const std::pair<float, size_t> &second) { return first.second == second.second; }), path.end());
    path.front().first = 0.0f;
    path.back().first = 1.0f;

    // One segment between every two neighboring intersections, cut from the Bezier of the new road
    Bezier rest = roadBezier(a.position, road, b.position);
    float consumed = 0.0f;
    for (size_t j = 0; j + 1 < path.size(); j++)
    {
        Bezier piece = rest;
        if (j + 2 < path.size()) splitBezier(rest, (path[j + 1].first - consumed) / (1.0f - consumed), piece, rest);
        consumed = path[j + 1].first;

        RoadSegment segment;
        segment.from = path[j].second;
        segment.to = path[j + 1].second;
        segment.road = road;
        segment.road.controls = { piece.points[1], piece.points[2] };
        const size_t same = SameRoadBetween(segment);
        if (same == SIZE_MAX) AddSegment(segment);
        else
        {
            // A road drawn over a road between the same intersections keeps that road, which takes the older generation and the larger
            // width
            segments[same].road.generation = std::min(segments[same].road.generation, road.generation);
            segments[same].road.width = fmaxf(segments[same].road.width, road.width);
            RefreshSegment(same);
        }
    }
    RefreshNodes();
    return true;
}

bool RoadGraph::CanMoveNode(size_t node, Vector2 position) const
{
    // Room kept between the intersection and other roads and intersections, besides their half widths
    constexpr float CLEARANCE = 0.5f;

    std::array<Polyline, MAX_MOVED_ROADS> lines;
    std::array<size_t, MAX_MOVED_ROADS> incident = {};
    float width = 0.0f;
    const size_t count = std::isfinite(position.x) && std::isfinite(position.y) ? PreviewMovedNode(node, position, lines, incident, width) : 0;
    if (count == 0) return false;

    for (size_t i = 0; i < count; i++)
    {
        if (Vector2Distance(lines[i].points[0], lines[i].points[lines[i].count - 1]) < MIN_ROAD_LENGTH || curveTouchesItself(lines[i], segments[incident[i]].road.width)) return false;

        for (size_t j = i + 1; j < count; j++)
        {
            const float gap = segments[incident[i]].road.width + segments[incident[j]].road.width;
            if (linesCross(samples(lines[i]), samples(lines[j]), false) || curvesRunAlong(lines[i], lines[j], gap, 0.5f * gap)) return false;
        }
    }

    for (size_t n = 0; n < nodes.size(); n++)
    {
        if (n != node && Vector2Distance(nodes[n], position) < 0.5f * (nodeWidth[node] + nodeWidth[n]) + CLEARANCE) return false;
    }

    // A road farther from the box of the moved roads than the widths of two roads neither crosses them nor runs along them
    Box movedBox;
    for (size_t i = 0; i < count; i++)
    {
        const Box box = boxOf(samples(lines[i]));
        movedBox = { Vector2Min(movedBox.min, box.min), Vector2Max(movedBox.max, box.max) };
    }
    for (size_t s = 0; s < segments.size(); s++)
    {
        const RoadSegment &other = segments[s];
        if (std::find(incident.begin(), incident.begin() + (long)count, s) != incident.begin() + (long)count) continue;

        const float reach = 0.5f * (nodeWidth[node] + other.road.width) + CLEARANCE;
        for (int i = 0; i + 1 < other.line.count && contains(inflate(other.box, reach), position); i++)
        {
            if (pointSegmentDistance(position, other.line.points[i], other.line.points[i + 1]) < reach) return false;
        }
        if (!overlap(inflate(movedBox, nodeWidth[node] + other.road.width), other.box)) continue;

        // Its roads do not cross the road, and do not run alongside it unless they meet at an end
        for (size_t i = 0; i < count; i++)
        {
            const RoadSegment &moved = segments[incident[i]];
            const bool adjacent = other.from == moved.from || other.from == moved.to || other.to == moved.from || other.to == moved.to;
            const float gap = moved.road.width + other.road.width;
            if (linesCross(samples(lines[i]), samples(other.line), false) || (!adjacent && curvesRunAlong(lines[i], other.line, gap, 0.5f * gap))) return false;
        }
    }
    return true;
}

bool RoadGraph::MoveNode(size_t node, Vector2 position)
{
    if (!CanMoveNode(node, position)) return false;

    nodes[node] = position;
    for (size_t s = 0; s < segments.size(); s++)
    {
        if (segments[s].from == node || segments[s].to == node) RefreshSegment(s);
    }
    return true;
}

void RoadGraph::DrawMovedNode(size_t node, Vector2 position, Color color) const
{
    std::array<Polyline, MAX_MOVED_ROADS> lines;
    std::array<size_t, MAX_MOVED_ROADS> incident = {};
    float width = 0.0f;
    const size_t count = PreviewMovedNode(node, position, lines, incident, width);
    for (size_t i = 0; i < count; i++) DrawSplineLinear(lines[i].points.data(), lines[i].count, width, color);
    if (count > 0) DrawCircleV(position, width * 0.5f, color);
}

bool RoadGraph::RemoveSegments(std::vector<size_t> indices)
{
    // From the last one on, so that the indices still to go stay where they are
    std::sort(indices.begin(), indices.end(), [](size_t first, size_t second) { return first > second; });
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    std::vector<size_t> ends;
    for (size_t s : indices)
    {
        ends.push_back(segments[s].from);
        ends.push_back(segments[s].to);
        segments.erase(segments.begin() + (long)s);
    }
    for (size_t node : ends) MergeStraightRoads(node);
    RefreshNodes();
    return !ends.empty();
}

bool RoadGraph::RemoveHanging(unsigned generation, bool deadEnds)
{
    bool removed = false;
    for (bool found = true; found; removed = removed || found)
    {
        // Every road joins the sets of its ends, a set with a road of another generation is reached
        std::vector<size_t> sets(nodes.size());
        for (size_t n = 0; n < sets.size(); n++) sets[n] = n;
        std::vector<char> reached(nodes.size(), 0);
        for (const RoadSegment &segment : segments) sets[findSet(sets, segment.from)] = findSet(sets, segment.to);
        for (const RoadSegment &segment : segments) reached[findSet(sets, segment.from)] |= segment.road.generation != generation ? 1 : 0;
        std::vector<size_t> hanging;
        for (size_t s = 0; s < segments.size(); s++)
        {
            const RoadSegment &segment = segments[s];
            const bool deadEnd = deadEnds && (nodeDegree[segment.from] == 1 || nodeDegree[segment.to] == 1);
            if (segment.road.generation == generation && (deadEnd || !reached[findSet(sets, segment.from)])) hanging.push_back(s);
        }
        found = RemoveSegments(hanging);
    }
    return removed;
}

void RoadGraph::Crossings(const Polyline &line, unsigned generation, std::vector<float> &params) const
{
    const Box box = boxOf(samples(line));
    std::vector<Hit> hits;
    for (size_t s = 0; s < segments.size(); s++)
    {
        if (segments[s].road.generation != generation && overlap(box, segments[s].box)) lineHits(line, segments[s].line, s, 0.0f, hits);
    }
    for (const Hit &hit : hits) params.push_back(hit.along);
}

void RoadGraph::Draw(Box view, float worldPerPixel) const
{
    constexpr Color ROAD_COLOR = { 26, 29, 36, 255 };
    constexpr Color EDGE_COLOR = { 118, 130, 156, 255 };
    constexpr Color LINE_COLOR = { 150, 160, 182, 90 };

    // In pixels on the screen except MAX_EDGE, the widest edge in the world
    constexpr float EDGE_PIXELS = 1.3f;
    constexpr float MAX_EDGE = 0.35f;
    constexpr float MIN_EDGE_ROAD_PIXELS = 3.0f;
    constexpr float MIN_LINE_ROAD_PIXELS = 7.0f;
    constexpr float DASH_PIXELS = 7.0f;
    constexpr float GAP_PIXELS = 9.0f;
    constexpr float LINE_PIXELS = 1.2f;

    const float edge = fminf(EDGE_PIXELS * worldPerPixel, MAX_EDGE);
    std::array<Vector2, 2 * CURVE_POINTS> wide = {};
    for (const RoadSegment &segment : segments)
    {
        if (segment.road.width < MIN_EDGE_ROAD_PIXELS * worldPerPixel || !overlap(segment.box, view)) continue;
        for (int i = 0; i < 2 * segment.line.count; i++) wide[i] = segment.ribbon[i] + Vector2Normalize(segment.ribbon[i] - segment.line.points[i / 2]) * edge;
        DrawTriangleStrip(wide.data(), 2 * segment.line.count, EDGE_COLOR);
    }
    DrawDisks(view, worldPerPixel, edge, 0.5f * MIN_EDGE_ROAD_PIXELS, EDGE_COLOR);

    for (const RoadSegment &segment : segments)
    {
        if (overlap(segment.box, view)) DrawTriangleStrip(segment.ribbon.data(), 2 * segment.line.count, ROAD_COLOR);
    }
    DrawDisks(view, worldPerPixel, 0.0f, 1.0f, ROAD_COLOR);

    // The middle line stops before an intersection or a dead end, not at a node between just two roads
    const float gap = GAP_PIXELS * worldPerPixel;
    for (const RoadSegment &segment : segments)
    {
        if (segment.road.width < MIN_LINE_ROAD_PIXELS * worldPerPixel || !overlap(segment.box, view)) continue;

        const float startSkip = nodeDegree[segment.from] == 2 ? 0.0f : nodeWidth[segment.from] * 0.5f + gap * 0.5f;
        const float endSkip = nodeDegree[segment.to] == 2 ? 0.0f : nodeWidth[segment.to] * 0.5f + gap * 0.5f;
        drawDashes(segment.line, startSkip, endSkip, DASH_PIXELS * worldPerPixel, gap, LINE_PIXELS * worldPerPixel, LINE_COLOR);
    }
}

void RoadGraph::DrawSegmentHighlight(size_t index, Color color) const
{
    DrawTriangleStrip(segments[index].ribbon.data(), 2 * segments[index].line.count, color);
}

void RoadGraph::TraceParcels(std::vector<Vector2> &points, std::vector<size_t> &sizes) const
{
    constexpr float MIN_AREA = 0.01f;

    points.clear();
    sizes.clear();

    // Halfedges sorted by the node they leave and by their angle, last[n] is the last one of node n in that order
    std::vector<float> angles(2 * segments.size());
    std::vector<size_t> order(angles.size());
    for (size_t h = 0; h < angles.size(); h++)
    {
        const Polyline &line = segments[h / 2].line;
        const Vector2 direction = (h & 1) ? line.points[line.count - 2] - line.points[line.count - 1] : line.points[1] - line.points[0];
        angles[h] = atan2f(direction.y, direction.x);
        order[h] = h;
    }
    std::sort(order.begin(), order.end(), [&](size_t first, size_t second) { return std::make_tuple(Origin(first), angles[first], first) < std::make_tuple(Origin(second), angles[second], second); });
    std::vector<size_t> place(order.size());
    std::vector<size_t> last(nodes.size(), 0);
    for (size_t i = 0; i < order.size(); i++)
    {
        place[order[i]] = i;
        last[Origin(order[i])] = i;
    }

    // Walks wait in parts, each followed by its length
    std::vector<char> visited(order.size(), 0);
    std::vector<size_t> walk;
    std::vector<size_t> parts;
    for (size_t first = 0; first < order.size(); first++)
    {
        // The next road clockwise is the one before the way back in the sorted order, or the last one of its node
        for (size_t h = first; !visited[h];)
        {
            visited[h] = 1;
            parts.push_back(h);
            const size_t back = place[h ^ 1];
            h = order[back > 0 && Origin(order[back - 1]) == Origin(order[back]) ? back - 1 : last[Origin(order[back])]];
        }
        parts.push_back(parts.size());

        // Walks around the outside and around holes run clockwise, their area is not positive. Parts a walk leaves out are walks too
        while (!parts.empty())
        {
            const size_t count = parts.back();
            walk.assign(parts.end() - 1 - (long)count, parts.end() - 1);
            parts.resize(parts.size() - 1 - count);
            if (WalkArea(walk, 0, walk.size()) <= MIN_AREA) continue;

            SimplifyWalk(walk, parts);
            const size_t before = points.size();
            for (size_t h : walk)
            {
                const Polyline &line = segments[h / 2].line;
                for (int i = 0; i + 1 < line.count; i++) points.push_back(line.points[(h & 1) ? line.count - 1 - i : i]);
            }
            sizes.push_back(points.size() - before);
        }
    }
}

void RoadGraph::DrawDisks(Box view, float worldPerPixel, float extra, float minRadiusPixels, Color color) const
{
    for (size_t n = 0; n < nodes.size(); n++)
    {
        const float radius = nodeWidth[n] * 0.5f;
        if (radius < minRadiusPixels * worldPerPixel || !overlap(inflate(Box{ nodes[n], nodes[n] }, radius + extra), view)) continue;
        DrawPoly(nodes[n], (int)Clamp(2.0f * radius / worldPerPixel, 8.0f, 36.0f), radius + extra, 0.0f, color);
    }
}

size_t RoadGraph::Origin(size_t halfedge) const
{
    return (halfedge & 1) ? segments[halfedge / 2].to : segments[halfedge / 2].from;
}

float RoadGraph::WalkArea(const std::vector<size_t> &walk, size_t from, size_t to) const
{
    if (from >= to) return 0.0f;

    // Summed around the first node of the walk, so that far out the sum keeps its precision. A part of the walk that comes back to
    // where it started has the same area around any point, and every sum compared with another one uses the same point
    const Vector2 origin = nodes[Origin(walk[0])];
    float area = 0.0f;
    for (size_t k = from; k < to; k++)
    {
        const Polyline &line = segments[walk[k] / 2].line;
        float sum = 0.0f;
        for (int i = 0; i + 1 < line.count; i++) sum += 0.5f * Vector2CrossProduct(line.points[i] - origin, line.points[i + 1] - origin);
        area += (walk[k] & 1) ? -sum : sum;
    }
    return area;
}

void RoadGraph::SimplifyWalk(std::vector<size_t> &walk, std::vector<size_t> &parts) const
{
    bool changed = true;
    while (changed && walk.size() >= 2)
    {
        changed = false;
        const size_t n = walk.size();
        for (size_t i = 0; i < n && !changed; i++)
        {
            if (walk[i] != (walk[(i + 1) % n] ^ 1)) continue;

            walk.erase(walk.begin() + (long)i, walk.begin() + (long)std::min(i + 2, n));
            if (i + 1 == n) walk.erase(walk.begin());
            changed = true;
        }
        for (size_t i = 0; i < n && !changed; i++)
        {
            for (size_t j = i + 1; j < n && !changed; j++)
            {
                if (Origin(walk[i]) != Origin(walk[j])) continue;

                // Turned to start at i the walk has the part up to j first, the part with the smaller area goes to parts
                std::rotate(walk.begin(), walk.begin() + (long)i, walk.end());
                const bool keepFirst = WalkArea(walk, 0, j - i) > WalkArea(walk, j - i, n);
                if (keepFirst) std::rotate(walk.begin(), walk.begin() + (long)(j - i), walk.end());
                const size_t drop = keepFirst ? n - (j - i) : j - i;
                parts.insert(parts.end(), walk.begin(), walk.begin() + (long)drop);
                parts.push_back(drop);
                walk.erase(walk.begin(), walk.begin() + (long)drop);
                changed = true;
            }
        }
    }
}

size_t RoadGraph::PreviewMovedNode(size_t node, Vector2 position, std::array<Polyline, MAX_MOVED_ROADS> &lines, std::array<size_t, MAX_MOVED_ROADS> &incident, float &width) const
{
    width = 0.0f;
    if (node >= nodes.size() || (size_t)nodeDegree[node] > MAX_MOVED_ROADS) return 0;

    size_t count = 0;
    for (size_t s = 0; s < segments.size(); s++)
    {
        const RoadSegment &segment = segments[s];
        if (segment.from != node && segment.to != node) continue;

        incident[count] = s;
        lines[count++] = roadLine(segment.from == node ? position : nodes[segment.from], segment.road, segment.to == node ? position : nodes[segment.to]);
        width = fmaxf(width, segment.road.width);
    }
    return count;
}

size_t RoadGraph::NodeAt(Vector2 point)
{
    const RoadAnchor anchor = Snap(point, MERGE_DISTANCE);
    if (anchor.kind == AnchorKind::Node) return anchor.node;
    if (anchor.kind == AnchorKind::Road) return SplitSegment(anchor.segment, anchor.t);
    nodes.push_back(point);
    return nodes.size() - 1;
}

void RoadGraph::AddSegment(const RoadSegment &segment)
{
    segments.push_back(segment);
    RefreshSegment(segments.size() - 1);
}

size_t RoadGraph::SplitSegment(size_t index, float t)
{
    // An intersection closer than this to where the segment is cut is the place it is cut at
    constexpr float SAME_PLACE = 0.05f;

    Bezier first;
    Bezier second;
    splitBezier(roadBezier(nodes[segments[index].from], segments[index].road, nodes[segments[index].to]), t, first, second);

    // A second intersection at the same place would make roads that run twice and crossings the parcels cannot follow
    size_t node = 0;
    if (!FindNode(first.points[3], SAME_PLACE, node) || node == segments[index].from || node == segments[index].to)
    {
        node = nodes.size();
        nodes.push_back(first.points[3]);
    }

    RoadSegment rest = segments[index];
    rest.from = node;
    rest.road.controls = { second.points[1], second.points[2] };
    segments[index].to = node;
    segments[index].road.controls = { first.points[1], first.points[2] };
    RefreshSegment(index);
    AddSegment(rest);
    return node;
}

size_t RoadGraph::SameRoadBetween(const RoadSegment &segment) const
{
    const Polyline line = roadLine(nodes[segment.from], segment.road, nodes[segment.to]);
    for (size_t s = 0; s < segments.size(); s++)
    {
        const RoadSegment &other = segments[s];
        if (!((other.from == segment.from && other.to == segment.to) || (other.from == segment.to && other.to == segment.from))) continue;

        // Every sample of each road is closer to the other road than MERGE_DISTANCE
        bool along = true;
        float t = 0.0f;
        Vector2 closest = { 0.0f, 0.0f };
        for (int i = 0; i < line.count && along; i++) along = closestOnLine(other.line, line.points[i], t, closest) < MERGE_DISTANCE * MERGE_DISTANCE;
        for (int i = 0; i < other.line.count && along; i++) along = closestOnLine(line, other.line.points[i], t, closest) < MERGE_DISTANCE * MERGE_DISTANCE;
        if (along) return s;
    }
    return SIZE_MAX;
}

void RoadGraph::RefreshSegment(size_t index)
{
    RoadSegment &segment = segments[index];
    segment.line = roadLine(nodes[segment.from], segment.road, nodes[segment.to]);
    const Vector2 *p = segment.line.points.data();
    const int count = segment.line.count;
    for (int i = 0; i < count; i++)
    {
        // The normals of the pieces before and after the sample, an end has only one piece
        const int previous = std::max(i - 1, 0);
        const int next = std::min(i + 1, count - 1);
        const Vector2 before = leftNormal(p[previous], p[previous + 1]);
        const Vector2 after = leftNormal(p[next - 1], p[next]);
        const Vector2 normal = Vector2Normalize(before + after);
        const float offset = 0.5f * segment.road.width / fmaxf(Vector2DotProduct(normal, after), 0.5f);
        segment.ribbon[2 * i] = p[i] + normal * offset;
        segment.ribbon[2 * i + 1] = p[i] - normal * offset;
    }
    segment.box = boxOf(std::span<const Vector2>(segment.ribbon.data(), (size_t)(2 * count)));
}

void RoadGraph::RefreshNodes()
{
    nodeWidth.assign(nodes.size(), 0.0f);
    nodeDegree.assign(nodes.size(), 0);
    for (const RoadSegment &segment : segments)
    {
        for (size_t node : { segment.from, segment.to })
        {
            nodeWidth[node] = fmaxf(nodeWidth[node], segment.road.width);
            nodeDegree[node]++;
        }
    }

    std::vector<size_t> renumbered(nodes.size(), SIZE_MAX);
    size_t kept = 0;
    for (size_t n = 0; n < nodes.size(); n++)
    {
        if (nodeDegree[n] == 0) continue;

        renumbered[n] = kept;
        nodes[kept] = nodes[n];
        nodeWidth[kept] = nodeWidth[n];
        nodeDegree[kept++] = nodeDegree[n];
    }
    nodes.resize(kept);
    nodeWidth.resize(kept);
    nodeDegree.resize(kept);
    for (RoadSegment &segment : segments)
    {
        segment.from = renumbered[segment.from];
        segment.to = renumbered[segment.to];
    }
}

void RoadGraph::MergeStraightRoads(size_t node)
{
    constexpr float COLLINEAR = 0.0001f;

    std::array<size_t, 3> incident = {};
    size_t count = 0;
    for (size_t s = 0; s < segments.size() && count < 3; s++)
    {
        if (segments[s].from == node || segments[s].to == node) incident[count++] = s;
    }
    if (count != 2) return;

    RoadSegment &first = segments[incident[0]];
    const RoadSegment &second = segments[incident[1]];
    if (first.road.curved || second.road.curved || first.road.width != second.road.width || first.road.generation != second.road.generation) return;

    const size_t endA = first.from == node ? first.to : first.from;
    const size_t endB = second.from == node ? second.to : second.from;
    const Vector2 toA = nodes[endA] - nodes[node];
    const Vector2 toB = nodes[endB] - nodes[node];
    if (endA == endB || Vector2DotProduct(toA, toB) >= 0.0f || fabsf(Vector2CrossProduct(toA, toB)) > COLLINEAR * Vector2Length(toA) * Vector2Length(toB)) return;

    first.from = endA;
    first.to = endB;
    RefreshSegment(incident[0]);
    segments.erase(segments.begin() + (long)incident[1]);
}

// ---- Parcels

void BoxGrid::Build(std::span<const Box> boxes)
{
    constexpr int MAX_COLUMNS = 64;

    bounds = Box();
    for (const Box &box : boxes) bounds = { Vector2Min(bounds.min, box.min), Vector2Max(bounds.max, box.max) };
    columns = std::clamp((int)sqrtf((float)boxes.size()), 1, MAX_COLUMNS);
    cellSize = fmaxf(fmaxf(bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y), 1.0f) / (float)columns;

    // Counts the boxes of every cell, every cell starts where the cells before it end, then the boxes go in in their order
    cellStart.assign((size_t)(columns * columns) + 1, 0);
    for (const Box &box : boxes)
    {
        const GridRange range = Range(box);
        for (int y = range.y0; y <= range.y1; y++)
        {
            for (int x = range.x0; x <= range.x1; x++) cellStart[(size_t)(y * columns + x) + 1]++;
        }
    }
    for (size_t c = 1; c < cellStart.size(); c++) cellStart[c] += cellStart[c - 1];

    std::vector<size_t> filled(cellStart.begin(), cellStart.end() - 1);
    items.resize(cellStart.back());
    for (size_t i = 0; i < boxes.size(); i++)
    {
        const GridRange range = Range(boxes[i]);
        for (int y = range.y0; y <= range.y1; y++)
        {
            for (int x = range.x0; x <= range.x1; x++) items[filled[(size_t)(y * columns + x)]++] = i;
        }
    }
}

GridRange BoxGrid::Range(Box box) const
{
    return { Index(box.min.x, bounds.min.x), Index(box.min.y, bounds.min.y), Index(box.max.x, bounds.min.x), Index(box.max.y, bounds.min.y) };
}

std::span<const size_t> BoxGrid::At(int x, int y) const
{
    const size_t cell = (size_t)(y * columns + x);
    return { items.data() + cellStart[cell], cellStart[cell + 1] - cellStart[cell] };
}

std::span<const size_t> BoxGrid::At(Vector2 p) const
{
    return At(Index(p.x, bounds.min.x), Index(p.y, bounds.min.y));
}

int BoxGrid::Index(float value, float origin) const
{
    return (int)Clamp(floorf((value - origin) / cellSize), 0.0f, (float)(columns - 1));
}

void ParcelMesh::Build(const RoadGraph &graph)
{
    // Every outline is a parcel, cut into triangles once per rebuild instead of every frame. The outlines keep their order in corners
    std::vector<size_t> sizes;
    graph.TraceParcels(corners, sizes);
    shapes.clear();
    triangles.clear();
    for (size_t i = 0, first = 0; i < sizes.size(); first += sizes[i++])
    {
        const std::span<const Vector2> points(corners.data() + first, sizes[i]);
        const size_t firstTriangle = triangles.size();
        triangulate(points, triangles);
        shapes.push_back({ .parcel = ParcelAttributes(), .area = signedArea(points), .box = boxOf(points), .probe = interiorPoint(points), .firstCorner = first,
            .cornerCount = sizes[i], .firstTriangle = firstTriangle, .triangleCount = triangles.size() - firstTriangle });
    }
    std::stable_sort(shapes.begin(), shapes.end(), [](const ParcelShape &a, const ParcelShape &b) { return a.area > b.area; });
    std::vector<Box> boxes;
    for (const ParcelShape &shape : shapes) boxes.push_back(shape.box);
    grid.Build(boxes);

    // The middle of a parcel around another parcel, like the one around a closed road, can be in the inner parcel. Its inside
    // point moves next to one of its sides then
    for (size_t i = 0; i < shapes.size(); i++)
    {
        const std::span<const Vector2> outline = Outline(i);
        size_t at = i;
        for (size_t k = 0; k < outline.size() && !(Find(shapes[i].probe, at) && at == i); k++)
        {
            shapes[i].probe = Vector2Lerp(outline[k], outline[(k + 1) % outline.size()], 0.5f) + leftNormal(outline[k], outline[(k + 1) % outline.size()]) * 0.25f;
        }
    }
}

std::span<const Vector2> ParcelMesh::Outline(size_t index) const
{
    return { corners.data() + shapes[index].firstCorner, shapes[index].cornerCount };
}

bool ParcelMesh::Find(Vector2 point, size_t &index) const
{
    bool found = false;
    for (size_t i : grid.At(point))
    {
        if ((found && i < index) || !contains(shapes[i].box, point) || !insidePolygon(Outline(i), point)) continue;

        index = i;
        found = true;
    }
    return found;
}

void ParcelMesh::Draw(size_t index, Color color) const
{
    const ParcelShape &shape = shapes[index];
    for (size_t t = shape.firstTriangle; t + 2 < shape.firstTriangle + shape.triangleCount; t += 3) DrawTriangle(triangles[t], triangles[t + 1], triangles[t + 2], color);
}

// ---- City

City::City() : mesh(std::make_unique<ParcelMesh>()), oldMesh(std::make_unique<ParcelMesh>())
{}

City::~City() = default;

void City::Draw(const Camera2D &camera) const
{
    constexpr Color LOT_COLOR = { 214, 222, 238, 64 };
    constexpr float MIN_LOT_PIXELS = 3.0f;

    // Muted colors for the dark theme, the parcel types are told apart by their hue
    constexpr Color STYLE_COLORS[] = { { 74, 88, 112, 255 }, { 66, 104, 90, 255 }, { 100, 86, 128, 255 }, { 124, 88, 84, 255 } };

    // The visible part of the world, the camera never rotates
    const Box view = { GetScreenToWorld2D({ 0.0f, 0.0f }, camera), GetScreenToWorld2D({ (float)GetScreenWidth(), (float)GetScreenHeight() }, camera) };
    const float worldPerPixel = 1.0f / camera.zoom;

    // Parcels follow the middle lines of the roads, so curved roads cut them right
    for (size_t i = 0; i < mesh->shapes.size(); i++)
    {
        if (overlap(mesh->shapes[i].box, view)) mesh->Draw(i, STYLE_COLORS[(int)mesh->shapes[i].parcel.style]);
    }

    // Lots are thin outlines, skipped when they are too small to see
    for (size_t l = 0; l < lotShapes.size(); l++)
    {
        const Box box = lotShapes[l].box;
        if (!overlap(box, view) || fmaxf(box.max.x - box.min.x, box.max.y - box.min.y) < MIN_LOT_PIXELS * worldPerPixel) continue;

        const std::span<const Vector2> corners = LotPoints(l);
        DrawLineStrip(corners.data(), (int)corners.size(), LOT_COLOR);
        DrawLineV(corners.back(), corners.front(), LOT_COLOR);
    }
    graph.Draw(view, worldPerPixel);
}

bool City::CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road, bool &alongRoad) const
{
    const bool valid = graph.CanAddRoad(a, b, road);
    alongRoad = valid && graph.RunsAlongRoad(a, b, road);
    return valid && !alongRoad;
}

bool City::AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road)
{
    bool alongRoad = false;
    if (!CanAddRoad(a, b, road, alongRoad) || !graph.AddRoad(a, b, road)) return false;

    RebuildParcels(true, std::nullopt);
    return true;
}

bool City::MoveNode(size_t node, Vector2 position)
{
    if (!graph.MoveNode(node, position)) return false;

    RebuildParcels(true, std::nullopt);
    return true;
}

bool City::RemoveRoadAt(Vector2 point, float radius)
{
    size_t index = 0;
    if (!graph.FindSegment(point, radius, index)) return false;

    // Without a generated road the grown parcel gets no streets, they would bring it back, and lots fill the place of a dead end
    const bool generated = graph.segments[index].road.generation != 0;
    const Vector2 freed = lineMiddle(graph.segments[index].line);
    graph.RemoveSegments({ index });
    RebuildParcels(!generated, freed);
    return true;
}

bool City::ApplyParcelTypeAt(Vector2 point, SubdivisionStyle style, bool wholeBlock)
{
    std::vector<size_t> parcels;
    std::vector<size_t> streets;
    if (!FindBlock(point, wholeBlock, parcels, streets)) return false;

    for (size_t i : parcels) mesh->shapes[i].parcel.style = style;
    if (!autoGenerate) return true;

    // Without its streets the block is one parcel again, parcels that change with it or lose lots are generated too
    graph.RemoveSegments(streets);
    RebuildParcels(true, point);
    return true;
}

bool City::ParcelAt(Vector2 point, size_t &index) const
{
    return mesh->Find(point, index);
}

bool City::FindBlock(Vector2 point, bool wholeBlock, std::vector<size_t> &parcels, std::vector<size_t> &streets) const
{
    size_t index = 0;
    parcels.clear();
    streets.clear();
    if (!ParcelAt(point, index)) return false;

    const unsigned block = mesh->shapes[index].parcel.block;
    parcels.push_back(index);
    if (!wholeBlock || !autoGenerate || block == 0) return true;

    // A street of the generation joins the sets of the parcels of the block on its two sides
    std::vector<size_t> sets(mesh->shapes.size());
    for (size_t i = 0; i < sets.size(); i++) sets[i] = i;
    std::vector<std::pair<size_t, size_t>> joins;
    for (size_t s = 0; s < graph.segments.size(); s++)
    {
        size_t a = 0;
        size_t b = 0;
        if (graph.segments[s].road.generation != block || !StreetSides(s, a, b) || mesh->shapes[a].parcel.block != block || mesh->shapes[b].parcel.block != block) continue;

        sets[findSet(sets, a)] = findSet(sets, b);
        joins.push_back({ s, a });
    }
    for (size_t i = 0; i < mesh->shapes.size(); i++)
    {
        if (i != index && findSet(sets, i) == findSet(sets, index)) parcels.push_back(i);
    }
    for (const std::pair<size_t, size_t> &join : joins)
    {
        if (findSet(sets, join.second) == findSet(sets, index)) streets.push_back(join.first);
    }
    return true;
}

void City::DrawParcels(const std::vector<size_t> &shapes, Color color) const
{
    for (size_t index : shapes) mesh->Draw(index, color);
}

bool City::StreetSides(size_t street, size_t &left, size_t &right) const
{
    // How far from the middle of a road its two sides are looked at
    constexpr float SIDE_DISTANCE = 0.7f;

    const Polyline &line = graph.segments[street].line;
    const Vector2 side = leftNormal(line.points[line.count / 2 - 1], line.points[line.count / 2]) * SIDE_DISTANCE;
    return ParcelAt(lineMiddle(line) + side, left) && ParcelAt(lineMiddle(line) - side, right);
}

std::span<const Vector2> City::LotPoints(size_t lot) const
{
    return { lotPoints.data() + lotShapes[lot].first, lotShapes[lot].count };
}

void City::BuildParcelMesh()
{
    // The parcels from before stay to hand their settings on, the ones before them make room
    std::swap(mesh, oldMesh);
    mesh->Build(graph);
    meshVersion = ++gMeshVersion;
    for (ParcelShape &shape : mesh->shapes)
    {
        size_t index = 0;
        shape.parcel.style = newParcelStyle;
        if (!oldMesh->Find(shape.probe, index)) continue;

        const ParcelShape &before = oldMesh->shapes[index];
        shape.parcel = before.parcel;
        shape.changed = !(fabsf(before.area - shape.area) <= 0.001f * shape.area && Vector2Distance(before.box.min, shape.box.min) < 0.05f &&
            Vector2Distance(before.box.max, shape.box.max) < 0.05f);
    }
}

void City::RebuildParcels(bool withStreets, std::optional<Vector2> freed)
{
    BuildParcelMesh();
    std::vector<char> lost = PruneLots();
    if (!autoGenerate) return;

    // Generating rebuilds the parcels, they are found again by their inside points. One that kept its shape but lost lots gets new lots
    // and the streets they need, not the streets of its type
    std::vector<Vector2> probes;
    std::vector<Vector2> refills;
    for (size_t i = 0; i < mesh->shapes.size(); i++)
    {
        if (mesh->shapes[i].changed) probes.push_back(mesh->shapes[i].probe);
        else if (lost[i] && mesh->shapes[i].parcel.block != 0) refills.push_back(mesh->shapes[i].probe);
    }
    size_t at = 0;
    if (freed && ParcelAt(*freed, at) && !mesh->shapes[at].changed) probes.push_back(mesh->shapes[at].probe);
    for (Vector2 probe : probes) GenerateAt(probe, withStreets);
    for (Vector2 probe : refills) GenerateAt(probe, false);

    // The new roads can run through lots around them, the parcels that lose lots get new ones without streets
    lost = PruneLots();
    for (size_t i = 0; i < lost.size(); i++)
    {
        if (lost[i] && mesh->shapes[i].parcel.block != 0) FillLots(i, mesh->shapes[i].parcel.block * 7919u + (unsigned)i);
    }
}

void City::GenerateAt(Vector2 point, bool withStreets)
{
    size_t index = 0;
    if (!ParcelAt(point, index)) return;

    // A parcel where roads cross without an intersection loses the loops they make, parcels that are still not simple stay as they are
    std::vector<Vector2> outline(mesh->Outline(index).begin(), mesh->Outline(index).end());
    if (!polygonIsSimple(outline)) untangle(outline);
    if (!polygonIsSimple(outline)) return;

    // The others lose the lots from before and the streets that earlier generations left inside, the dead ends to those lots
    std::vector<size_t> inside;
    for (size_t s = 0; s < graph.segments.size(); s++)
    {
        if (graph.segments[s].road.generation != 0 && parcelAround(graph, *mesh, s) == index) inside.push_back(s);
    }
    if (graph.RemoveSegments(inside))
    {
        BuildParcelMesh();
        if (!ParcelAt(point, index)) return;
    }

    // A parcel that gets streets is the middle of its pattern, one that only gets lots stays in the pattern it was generated in. It is
    // generated with the settings of its type as they are now
    const unsigned generation = nextGeneration++;
    ParcelAttributes &parcel = mesh->shapes[index].parcel;
    if (withStreets || parcel.block == 0) parcel.center = interiorPoint(outline);
    parcel.block = generation;
    parcel.settings = parcelSettings[(size_t)parcel.style];
    const ParcelAttributes attributes = parcel;

    // Streets of the type cut a large parcel into blocks, the ones that end in nothing would run through lots
    std::vector<Street> streets;
    if (withStreets) generateStreetPattern(outline, attributes, generation, streets);
    const bool patterned = AddStreets(streets, generation);
    if (graph.RemoveHanging(generation, true) || patterned) BuildParcelMesh();

    // The streets can take parcels around the parcel into its blocks, streets of earlier generations that end in a block go then too
    inside.clear();
    for (size_t s = 0; s < graph.segments.size(); s++)
    {
        const unsigned streetGeneration = graph.segments[s].road.generation;
        const size_t around = parcelAround(graph, *mesh, s);
        if (streetGeneration != 0 && streetGeneration != generation && around != SIZE_MAX && mesh->shapes[around].parcel.block == generation) inside.push_back(s);
    }
    if (graph.RemoveSegments(inside))
    {
        graph.RemoveHanging(generation, true);
        BuildParcelMesh();
    }

    // Every block gets lots and the streets they need, roads that end in a block, like drawn dead ends, cut it first
    std::vector<Polyline> ends;
    std::vector<Vector2> points;
    std::vector<size_t> sizes;
    streets.clear();
    for (size_t i = 0, k = 0; i < mesh->shapes.size(); i++)
    {
        if (mesh->shapes[i].parcel.block != generation) continue;

        roadsInside(graph, *mesh, i, ends);
        LotSubdivider(attributes, generation * 7919u + (unsigned)k++).Run(mesh->Outline(i), ends, points, sizes, streets);
    }
    if (AddStreets(streets, generation)) BuildParcelMesh();

    // The lots from before in the blocks make room, also in the parcels the blocks took in. The new lots that ended up in a block and
    // are on a road stay
    std::vector<char> blocks(mesh->shapes.size(), 0);
    for (size_t i = 0; i < blocks.size(); i++) blocks[i] = mesh->shapes[i].parcel.block == generation;
    RemoveLotsIn(blocks);
    const BoxGrid roads = roadGrid(graph);
    std::vector<char> filled(mesh->shapes.size(), 0);
    for (size_t l = 0, first = 0; l < sizes.size(); first += sizes[l++])
    {
        const std::span<const Vector2> lot(points.data() + first, sizes[l]);
        size_t at = 0;
        if (!ParcelAt(interiorPoint(lot), at) || !blocks[at] || !onRoad(graph, roads, *mesh, at, lot)) continue;

        AddLot(lot);
        filled[at] = 1;
    }

    // A block that the streets of the lots cut off, or that lost its lots to roads through them, gets lots of its own without streets. A
    // block smaller than a lot is left to the roads around it
    for (size_t i = 0; i < mesh->shapes.size(); i++)
    {
        if (blocks[i] && !filled[i] && mesh->shapes[i].area >= attributes.settings.minParcelArea) FillLots(i, generation * 7919u + (unsigned)i);
    }
}

bool City::AddStreets(const std::vector<Street> &streets, unsigned generation)
{
    constexpr float STREET_SNAP = 0.3f;
    constexpr float ALONG_ROAD = 0.5f;

    bool added = false;
    std::vector<float> cuts;
    std::vector<float> stretches;
    for (const Street &street : streets)
    {
        const Vector2 a = graph.Snap(street.a, STREET_SNAP).position;
        const Vector2 b = graph.Snap(street.b, STREET_SNAP).position;
        const Bezier curve = roadBezier(a, street.road, b);
        cuts.assign({ 0.0f, 1.0f });
        graph.Crossings(roadLine(a, street.road, b), generation, cuts);
        std::sort(cuts.begin(), cuts.end());
        for (size_t k = 0; k + 1 < cuts.size(); k++)
        {
            size_t at = 0;
            if (!ParcelAt(bezierPoint(curve, 0.5f * (cuts[k] + cuts[k + 1])), at) || mesh->shapes[at].parcel.block != generation) continue;

            // Along a road a piece would be a second road on top of it, like a ring of a pattern laid again, the rest is laid
            const Bezier piece = subBezier(curve, cuts[k], cuts[k + 1]);
            offRoads(graph, piece, ALONG_ROAD, stretches);
            for (size_t r = 0; r + 1 < stretches.size(); r += 2)
            {
                const Bezier part = stretches[r] == 0.0f && stretches[r + 1] == 1.0f ? piece : subBezier(piece, stretches[r], stretches[r + 1]);
                RoadAttributes road = street.road;
                road.generation = generation;
                road.controls = { part.points[1], part.points[2] };
                added = graph.AddRoad(graph.Snap(part.points[0], STREET_SNAP), graph.Snap(part.points[3], STREET_SNAP), road) || added;
            }
        }
    }
    return graph.RemoveHanging(generation, false) || added;
}

void City::AddLot(std::span<const Vector2> points)
{
    lotShapes.push_back({ lotPoints.size(), points.size(), boxOf(points), interiorPoint(points) });
    lotPoints.insert(lotPoints.end(), points.begin(), points.end());
}

void City::KeepLots(const std::vector<char> &keep)
{
    std::vector<Vector2> points;
    std::vector<LotShape> shapes;
    for (size_t l = 0; l < lotShapes.size(); l++)
    {
        if (!keep[l]) continue;

        shapes.push_back(lotShapes[l]);
        shapes.back().first = points.size();
        points.insert(points.end(), LotPoints(l).begin(), LotPoints(l).end());
    }
    lotPoints = std::move(points);
    lotShapes = std::move(shapes);
}

void City::RemoveLotsIn(const std::vector<char> &parcels)
{
    std::vector<char> keep(lotShapes.size(), 1);
    for (size_t l = 0; l < lotShapes.size(); l++)
    {
        size_t at = 0;
        keep[l] = !(ParcelAt(lotShapes[l].probe, at) && parcels[at]);
    }
    KeepLots(keep);
}

std::vector<char> City::PruneLots()
{
    const BoxGrid roads = roadGrid(graph);
    std::vector<char> keep(lotShapes.size(), 0);
    std::vector<char> lost(mesh->shapes.size(), 0);
    for (size_t l = 0; l < lotShapes.size(); l++)
    {
        size_t index = 0;
        const bool inParcel = ParcelAt(lotShapes[l].probe, index);
        keep[l] = inParcel && onRoad(graph, roads, *mesh, index, LotPoints(l));
        if (inParcel && !keep[l]) lost[index] = 1;
    }
    KeepLots(keep);
    return lost;
}

void City::FillLots(size_t index, unsigned seed)
{
    std::vector<Polyline> ends;
    roadsInside(graph, *mesh, index, ends);
    std::vector<char> parcels(mesh->shapes.size(), 0);
    parcels[index] = 1;
    RemoveLotsIn(parcels);

    std::vector<Vector2> points;
    std::vector<size_t> sizes;
    std::vector<Street> streets;
    LotSubdivider(mesh->shapes[index].parcel, seed).Run(mesh->Outline(index), ends, points, sizes, streets);
    const BoxGrid roads = roadGrid(graph);
    for (size_t l = 0, first = 0; l < sizes.size(); first += sizes[l++])
    {
        const std::span<const Vector2> lot(points.data() + first, sizes[l]);
        size_t at = 0;
        if (ParcelAt(interiorPoint(lot), at) && at == index && onRoad(graph, roads, *mesh, at, lot)) AddLot(lot);
    }
}

// ---- Editor

Editor::Editor()
{
    static_assert(std::size(BUTTONS) == ButtonKind::_Button_Count, "BUTTONS follows ButtonKind");
    static_assert(std::size(OPTIONS) == OptionKind::_Option_Count, "OPTIONS follows OptionKind");

    for (int i = 0; i < ButtonKind::_Button_Count; ++i) buttonIcons[i] = LoadTexture(BUTTONS[i].iconPath);
    for (int i = 0; i < OptionKind::_Option_Count; ++i) optionIcons[i] = LoadTexture(OPTIONS[i].iconPath);
}

Editor::~Editor()
{
    for (Texture2D &icon : buttonIcons) UnloadTexture(icon);
    for (Texture2D &icon : optionIcons) UnloadTexture(icon);
}

void Editor::DrawWorld(City &city, const Camera2D &camera)
{
    constexpr Color VALID_COLOR = { 74, 222, 148, 170 };
    constexpr Color INVALID_COLOR = { 248, 100, 110, 170 };
    constexpr Color MARKER_COLOR = { 96, 165, 250, 255 };
    constexpr float CLICK_PIXELS = 4.0f; // Press and release closer than this is a click, farther is a drag

    const RoadGraph &roads = city.graph;
    const Vector2 mouse = GetMousePosition();
    const Vector2 world = GetScreenToWorld2D(mouse, camera);
    const float snapRadius = 14.0f / camera.zoom;
    const float markerRadius = 7.0f / camera.zoom;
    const bool clicked = !mouseOverToolbar && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    // A right click without dragging cancels the road or the move, dragging pans the camera
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) rightPress = mouse;
    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) && Vector2Distance(mouse, rightPress) < CLICK_PIXELS) step = 0;

    if (selectedButton == None)
    {
        status = "Pick a tool or a parcel type";
        return;
    }

    // Each tool draws its preview from the state before the click, then handles the click
    switch (BUTTONS[selectedButton].tool)
    {
        case Tool::Road:
        {
            // Clicks: the start, the bend point when curved, then the end
            const bool curved = optionEnabled[Curved];
            const int endStep = curved ? 2 : 1;

            // The point a click would use, a ring when it snapped to a road or an intersection, a dot when free
            RoadAnchor hover;
            const bool hasHover = !mouseOverToolbar;
            if (hasHover)
            {
                hover = roads.Snap(world, snapRadius);
                if (hover.kind == AnchorKind::Free) DrawCircleV(hover.position, markerRadius * 0.4f, { 240, 244, 250, 200 });
                else DrawRing(hover.position, markerRadius * 0.7f, markerRadius, 0.0f, 360.0f, 24, MARKER_COLOR);
            }

            RoadAttributes road;
            bool valid = false;
            status = "Click anywhere to start a road, snap to a road or an intersection to connect";
            if (step > 0) DrawCircleV(start.position, markerRadius * 0.7f, MARKER_COLOR);
            if (step > 0 && step < endStep)
            {
                // Choosing the bend point, a plain guide line is enough
                DrawLineEx(start.position, world, 1.5f / camera.zoom, MARKER_COLOR);
                status = "Click to place the bend point";
            }
            else if (step == endStep)
            {
                const Vector2 end = hasHover ? hover.position : world;
                if (curved) road = bentRoad(start.position, bend, end);
                bool alongRoad = false;
                valid = hasHover && city.CanAddRoad(start, hover, road, alongRoad);
                const Polyline line = roadLine(start.position, road, end);
                DrawSplineLinear(line.points.data(), line.count, road.width, valid ? VALID_COLOR : INVALID_COLOR);
                if (curved) DrawRing(bend, markerRadius * 0.4f, markerRadius * 0.7f, 0.0f, 360.0f, 24, MARKER_COLOR);
                status = alongRoad ? "This road would run alongside another road, move it away or end it on that road" : "Click anywhere to finish   RMB click: cancel";
            }

            if (!clicked) break;
            if (step < endStep)
            {
                if (step == 0) start = hover;
                else bend = world;
                step++;
            }
            else if (valid && city.AddRoad(start, hover, road)) step = 0;
            break;
        }
        case Tool::Remove:
        {
            // The road a click would remove
            size_t road = 0;
            status = "Click a road to remove it, the parcels beside it merge or disappear";
            if (mouseOverToolbar || !roads.FindSegment(world, snapRadius, road)) break;

            roads.DrawSegmentHighlight(road, { 248, 100, 110, 190 });
            if (clicked) city.RemoveRoadAt(world, snapRadius);
            break;
        }
        case Tool::Move:
        {
            if (step == 0)
            {
                // A ring on the intersection under the mouse, pressing on it starts dragging it
                size_t node = 0;
                status = "Drag an intersection to move it, the roads that end at it follow";
                if (mouseOverToolbar || !roads.FindNode(world, snapRadius, node)) break;

                DrawRing(roads.nodes[node], markerRadius * 0.9f, markerRadius * 1.3f, 0.0f, 360.0f, 24, MARKER_COLOR);
                if (clicked)
                {
                    dragNode = node;
                    step = 1;
                }
                break;
            }

            // The dragged intersection follows the mouse with its roads, it moves when the button is let go
            const bool valid = roads.CanMoveNode(dragNode, world);
            roads.DrawMovedNode(dragNode, world, valid ? VALID_COLOR : INVALID_COLOR);
            DrawRing(world, markerRadius * 0.7f, markerRadius, 0.0f, 360.0f, 24, valid ? MARKER_COLOR : INVALID_COLOR);
            status = valid ? "Let go to move the intersection, its roads follow   RMB click: cancel" : "It cannot go there: its roads would cross or run alongside another road, or it is too close to one";

            if (!IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) break;

            // Letting go without dragging is just a click, nothing moves
            if (valid && Vector2Distance(world, roads.nodes[dragNode]) > CLICK_PIXELS / camera.zoom) city.MoveNode(dragNode, world);
            step = 0;
            break;
        }
        case Tool::Parcel:
        {
            const bool whole = !optionEnabled[OneParcel];
            const char *label = BUTTONS[selectedButton].label;
            status = whole ? TextFormat("%s: click a parcel to change the whole block it belongs to", label) : TextFormat("%s: click a parcel to change only that parcel, the streets around it stay", label);
            if (mouseOverToolbar) break;

            // The parcels a click would change, the parcel under the mouse or its whole block. The streets of the block are not shown
            size_t parcel = 0;
            const size_t key = city.ParcelAt(world, parcel) ? parcel : SIZE_MAX;
            if (key != hoverParcel || whole != hoverWhole || city.meshVersion != hoverVersion)
            {
                std::vector<size_t> streets;
                hoverParcel = key;
                hoverWhole = whole;
                hoverVersion = city.meshVersion;
                city.FindBlock(world, whole, hoverShapes, streets);
            }
            city.DrawParcels(hoverShapes, { 255, 255, 255, 48 });

            // The parcel, or its whole block, is generated again in the form of its new type
            if (clicked) city.ApplyParcelTypeAt(world, (SubdivisionStyle)(selectedButton - ParcelGrid), whole);
            break;
        }
    }
}

bool Editor::DoItem(Rectangle bounds, Rectangle iconArea, Vector2 labelPosition, Texture2D icon, int fontSize, const char *label, bool selected)
{
    const bool hovered = CheckCollisionPointRec(GetMousePosition(), bounds);
    if (hovered) mouseOverToolbar = true;
    if (selected) DrawRectangleRounded({ bounds.x - 2.0f, bounds.y - 2.0f, bounds.width + 4.0f, bounds.height + 4.0f }, 0.16f, 8, { 96, 165, 250, 255 });
    DrawRectangleRounded(bounds, 0.16f, 8, hovered ? Color{ 38, 42, 52, 245 } : Color{ 26, 29, 36, 240 });

    if (icon.id != 0)
    {
        // Scale to fit and center in the area
        const float scale = fminf(iconArea.width / icon.width, iconArea.height / icon.height);
        const float width = icon.width * scale;
        const float height = icon.height * scale;
        const Rectangle destination = { iconArea.x + (iconArea.width - width) * 0.5f, iconArea.y + (iconArea.height - height) * 0.5f, width, height };
        DrawTexturePro(icon, { 0.0f, 0.0f, (float)icon.width, (float)icon.height }, destination, { 0.0f, 0.0f }, 0.0f, WHITE);
    }

    DrawText(label, (int)labelPosition.x, (int)labelPosition.y, fontSize, selected ? Color{ 236, 240, 248, 255 } : Color{ 150, 158, 174, 255 });
    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void Editor::Draw()
{
    static constexpr float BUTTON_SIZE = 62.0f;
    static constexpr float OPTION_HEIGHT = 24.0f;
    static constexpr float PADDING = 7.0f; // Gap between items and padding inside them
    static constexpr int FONT_SIZE = 13;

    mouseOverToolbar = false;

    const float left = ((float)GetScreenWidth() - (float)ButtonKind::_Button_Count * BUTTON_SIZE) * 0.5f;
    float x = left;
    float y = (float)GetScreenHeight() - BUTTON_SIZE - 20.0f;

    // Buttons, icon above the label, an extra gap where the tool changes, a click cancels the current action
    for (int i = 0; i < ButtonKind::_Button_Count; ++i)
    {
        const ToolbarItem &button = BUTTONS[i];
        if (i > 0 && button.tool != BUTTONS[i - 1].tool) x += PADDING;

        const bool isSelected = selectedButton == i;
        const Rectangle bounds = { x, y, BUTTON_SIZE, BUTTON_SIZE };
        const Rectangle iconArea = { x + PADDING, y + PADDING, BUTTON_SIZE - 2.0f * PADDING, 30.0f };
        const Vector2 labelPosition = { x + (BUTTON_SIZE - MeasureText(button.label, FONT_SIZE)) * 0.5f, y + BUTTON_SIZE - FONT_SIZE - 2.0f };
        if (DoItem(bounds, iconArea, labelPosition, buttonIcons[i], FONT_SIZE, button.label, isSelected))
        {
            selectedButton = isSelected ? None : (ButtonKind)i;
            step = 0;
        }

        x += BUTTON_SIZE + PADDING;
    }

    // Options of the selected tool, icon left of the label, a click cancels the current action
    const float iconSize = OPTION_HEIGHT - 4.0f;
    x = left;
    y -= OPTION_HEIGHT + PADDING;
    for (int i = 0; i < OptionKind::_Option_Count; ++i)
    {
        const ToolbarItem &option = OPTIONS[i];
        if (selectedButton == None || option.tool != BUTTONS[selectedButton].tool) continue;

        const float labelX = x + PADDING + iconSize + PADDING;
        const float width = labelX - x + MeasureText(option.label, FONT_SIZE) + PADDING;
        const Rectangle bounds = { x, y, width, OPTION_HEIGHT };
        const Rectangle iconArea = { x + PADDING, y + (OPTION_HEIGHT - iconSize) * 0.5f, iconSize, iconSize };
        const Vector2 labelPosition = { labelX, y + (OPTION_HEIGHT - FONT_SIZE) * 0.5f };
        if (DoItem(bounds, iconArea, labelPosition, optionIcons[i], FONT_SIZE, option.label, optionEnabled[i]))
        {
            optionEnabled[i] = !optionEnabled[i];
            step = 0;
        }

        x += width + PADDING;
    }

    // Centered above the options, 18px text with 10px padding on the sides and 5px on top and bottom
    const float textWidth = (float)MeasureText(status, 18);
    const Rectangle box = { ((float)GetScreenWidth() - textWidth) * 0.5f - 10.0f, y - PADDING - 28.0f, textWidth + 20.0f, 28.0f };
    DrawRectangleRounded(box, 0.4f, 8, { 20, 22, 28, 225 });
    DrawText(status, (int)(box.x + 10.0f), (int)(box.y + 5.0f), 18, { 226, 230, 238, 255 });
}