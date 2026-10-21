#include "City.h"

#include <algorithm>
#include <cmath>

#include <raymath.h>


// Most corners of a polygon while it is split into lots
constexpr int MAX_CELL_POINTS = 256;

// Corners of an organic chord between its ends, it bends between them
constexpr int BEND_POINTS = 7;

// A block shorter than this gets no streets, at zero they would be no distance apart
constexpr float MIN_BLOCK_SIZE = 1.0f;

// Room kept from the middle line of a road, half the road and a strip of grass, and from the other sides of a lot
constexpr float STREET_CLEARANCE = 2.0f;
constexpr float SIDE_CLEARANCE = 0.6f;
constexpr float DRIVEWAY_HALF_WIDTH = 1.1f;

static constexpr Color VALID_COLOR = { 96, 165, 250, 180 };
static constexpr Color INVALID_COLOR = { 240, 96, 96, 180 };
static constexpr Color ANCHOR_COLOR = { 236, 240, 248, 255 };

// ---- Types

// Every road is a cubic Bezier from its first to its last point
struct Bezier
{
    std::array<Vector2, 4> points = {};
};

// A street of a pattern from a to b, curved when its road is
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

// Where a line crosses a road: the curve parameter along the line, the road and the curve parameter along the road
struct Hit
{
    float along = 0.0f;
    int road = 0;
    float t = 0.0f;
};

// An intersection along a new road
struct Stop
{
    float along = 0.0f;
    int node = 0;
};

// Where a side of a polygon crosses a line, t is the distance along the line. A crossing moved onto a corner has the side starting there
struct Crossing
{
    int side = 0;
    Vector2 point = { 0.0f, 0.0f };
    float t = 0.0f;
};

// Lowest and highest projection of points on an axis
struct Interval
{
    float low = FLT_MAX;
    float high = -FLT_MAX;
};

// Smallest rectangle around a polygon with one side along a side of the polygon
struct OrientedBox
{
    Vector2 axisLong = { 1.0f, 0.0f };
    Vector2 axisShort = { 0.0f, 1.0f };
    float length = 0.0f;
    float width = 0.0f;
};

// A candidate line for a split
struct Line
{
    Vector2 origin = { 0.0f, 0.0f };
    Vector2 direction = { 1.0f, 0.0f };
};

// A lot while a house is placed on it: its corners, whether each side runs along a road, and the roads near it, which can also end at a
// side that runs along none
struct Ground
{
    std::span<const Vector2> lot = {};
    std::span<const bool> streets = {};
    std::span<const Road> roads = {};
};

// A polygon while it is split into lots, street[i] is set when the side from corner i to the next one runs along a road
struct Cell
{
    std::array<Vector2, MAX_CELL_POINTS> points = {};
    std::array<bool, MAX_CELL_POINTS> street = {};
    int count = 0;

    // A corner at the place of the last one takes its place, false when the cell is full
    bool Add(Vector2 point, bool isStreet);
};

// How freely a cell is split into lots: within the settings with road for all the lots each part still has to give, the same with a street
// that brings the road, or with some road along both parts only
enum SplitRule
{
    SplitRule_Settings, SplitRule_Street, SplitRule_Lenient
};

// ---- Boxes

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

// ---- Curves

// Unit vector to the left of the way from a to b
static Vector2 leftNormal(Vector2 a, Vector2 b)
{
    return Vector2Normalize(Vector2{ a.y - b.y, b.x - a.x });
}

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
    for (int i = 0; i < CURVE_POINTS; ++i) line.points[i] = bezierPoint(curve, (float)i / (float)(CURVE_POINTS - 1));
    return line;
}

static std::span<const Vector2> samples(const Polyline &line)
{
    return { line.points.data(), (size_t)line.count };
}

// The middle of a line, a sample or halfway between the two middle samples
static Vector2 lineMiddle(const Polyline &line)
{
    // A line without samples or with more than there is room for has no middle
    if (line.count < 1 || line.count > CURVE_POINTS) return { 0.0f, 0.0f };

    const int middle = line.count / 2;
    return line.count % 2 == 1 ? line.points[middle] : Vector2Lerp(line.points[middle - 1], line.points[middle], 0.5f);
}

// The corner of a line moved by distance to its left, along the bisector of its two pieces. The tip of a way out and back moves ahead
static Vector2 offsetCorner(Vector2 previous, Vector2 corner, Vector2 next, float distance)
{
    const Vector2 after = leftNormal(corner, next);
    const Vector2 sum = leftNormal(previous, corner) + after;
    if (Vector2LengthSqr(sum) < 0.0001f) return corner + Vector2Normalize(corner - previous) * distance;

    const Vector2 normal = Vector2Normalize(sum);
    return corner + normal * (distance / fmaxf(Vector2DotProduct(normal, after), 0.5f));
}

// The outline of a road as a strip, with a right and a left point per sample. That order makes the triangles of the strip clockwise
// in the world, which raylib does not cull. Returns the number of points, none for a line that has no piece
static int ribbon(const Polyline &line, float halfWidth, std::array<Vector2, 2 * CURVE_POINTS> &strip)
{
    // The mirrored neighbors of the ends need a second sample, the strip has room for CURVE_POINTS
    if (line.count < 2 || line.count > CURVE_POINTS) return 0;

    const int last = line.count - 1;
    for (int i = 0; i <= last; ++i)
    {
        // An end has only one piece, its other neighbor is mirrored
        const Vector2 previous = i > 0 ? line.points[i - 1] : line.points[0] * 2.0f - line.points[1];
        const Vector2 next = i < last ? line.points[i + 1] : line.points[last] * 2.0f - line.points[last - 1];
        strip[2 * i] = offsetCorner(previous, line.points[i], next, -halfWidth);
        strip[2 * i + 1] = offsetCorner(previous, line.points[i], next, halfWidth);
    }
    return 2 * line.count;
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
    for (size_t i = 0; i < polygon.size(); ++i) best = fminf(best, pointSegmentDistance(p, polygon[i], polygon[(i + 1) % polygon.size()]));
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

// Whether a piece of the first line crosses a piece of the second other than at their ends
static bool linesCross(std::span<const Vector2> first, std::span<const Vector2> second)
{
    for (size_t i = 0; i + 1 < first.size(); ++i)
    {
        for (size_t j = 0; j + 1 < second.size(); ++j)
        {
            if (segmentsCross(first[i], first[i + 1], second[j], second[j + 1])) return true;
        }
    }
    return false;
}

// Positive when the corners run counter-clockwise. Summed around the first corner, so that small polygons far out keep their sign
static float signedArea(std::span<const Vector2> polygon)
{
    float twice = 0.0f;
    for (size_t i = 1; i + 1 < polygon.size(); ++i) twice += Vector2CrossProduct(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
    return 0.5f * twice;
}

// Ear clipping of a simple polygon of either winding into triangles of the same winding, three corner indices per triangle. Returns the
// index count. Without an ear the first convex corner goes anyway
static int triangulate(std::span<const Vector2> polygon, std::span<uint16_t> out)
{
    const int n = (int)polygon.size();
    if (n < 3 || n > MAX_PARCEL_CORNERS) return 0;

    // The corners left form a ring
    std::array<int, MAX_PARCEL_CORNERS> previous;
    std::array<int, MAX_PARCEL_CORNERS> next;
    for (int i = 0; i < n; ++i)
    {
        previous[i] = (i + n - 1) % n;
        next[i] = (i + 1) % n;
    }

    // Every search goes on from the corner after the last ear, where the next ear most likely is
    const float winding = signedArea(polygon) >= 0.0f ? 1.0f : -1.0f;
    int count = 0;
    int from = 0;
    for (int left = n; left >= 3 && count + 3 <= (int)out.size(); --left)
    {
        int clip = -1;
        for (int k = 0, i = from; k < left; ++k, i = next[i])
        {
            const Vector2 a = polygon[previous[i]];
            const Vector2 b = polygon[i];
            const Vector2 c = polygon[next[i]];
            if (Vector2CrossProduct(b - a, c - b) * winding <= 0.0f) continue;
            if (clip < 0) clip = i;

            // A corner out of the box of the triangle is not in it, which is quicker to tell
            const Box box = inflate(boxOf(std::array<Vector2, 3>{ a, b, c }), 0.001f);
            bool ear = true;
            for (int j = next[next[i]]; j != previous[i] && ear; j = next[j]) ear = !contains(box, polygon[j]) || !CheckCollisionPointTriangle(polygon[j], a, b, c);
            if (!ear) continue;

            clip = i;
            break;
        }

        // A ring without convex corners has no area left
        if (clip < 0) break;

        out[count++] = (uint16_t)previous[clip];
        out[count++] = (uint16_t)clip;
        out[count++] = (uint16_t)next[clip];
        next[previous[clip]] = next[clip];
        previous[next[clip]] = previous[clip];
        from = next[clip];
    }
    return count;
}

// A point well inside a simple polygon: its centroid when that is, else the middle of the first way straight across from the middle of a
// side that is long enough, or of the longest one. Close to a side along a road the point would fall out of the parcel traced there
static Vector2 interiorPoint(std::span<const Vector2> polygon)
{
    // Room from the sides that a centroid needs
    constexpr float MIN_DEPTH = 0.1f;

    if (polygon.empty()) return { 0.0f, 0.0f };

    const size_t n = polygon.size();
    const Vector2 origin = polygon[0];
    float twiceArea = 0.0f;
    Vector2 sum = { 0.0f, 0.0f };
    for (size_t i = 0; i < n; ++i)
    {
        const Vector2 a = polygon[i] - origin;
        const Vector2 b = polygon[(i + 1) % n] - origin;
        twiceArea += Vector2CrossProduct(a, b);
        sum += (a + b) * Vector2CrossProduct(a, b);
    }

    const Vector2 centroid = fabsf(twiceArea) > 0.000001f ? origin + sum / (3.0f * twiceArea) : origin;
    if (insidePolygon(polygon, centroid) && boundaryDistance(polygon, centroid) > MIN_DEPTH) return centroid;

    // The inside is on the left of the sides of a counter-clockwise polygon
    const float inward = twiceArea >= 0.0f ? 1.0f : -1.0f;
    Vector2 best = origin;
    float deepest = 0.0f;
    for (size_t i = 0; i < n && deepest < 2.0f * MIN_DEPTH; ++i)
    {
        const Vector2 start = Vector2Lerp(polygon[i], polygon[(i + 1) % n], 0.5f);
        const Vector2 normal = leftNormal(polygon[i], polygon[(i + 1) % n]) * inward;
        float across = FLT_MAX;
        for (size_t j = 0; j < n; ++j)
        {
            float u = 0.0f;
            float v = 0.0f;
            const bool meet = intersect(start, normal, polygon[j], polygon[(j + 1) % n] - polygon[j], u, v);

            if (j != i && meet && u > 0.0f && v >= 0.0f && v <= 1.0f) across = fminf(across, u);
        }
        if (across == FLT_MAX || across <= deepest) continue;

        deepest = across;
        best = start + normal * (0.5f * across);
    }
    return best;
}

// True for at least three corners when no two of them are at the same place and no two sides cross
static bool polygonIsSimple(std::span<const Vector2> points)
{
    constexpr float SAME_POINT = 0.001f;

    const size_t n = points.size();
    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = i + 1; j < n; ++j)
        {
            if (Vector2Distance(points[i], points[j]) < SAME_POINT) return false;
            if (j > i + 1 && !(i == 0 && j + 1 == n) && segmentsCross(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n])) return false;
        }
    }
    return n >= 3;
}

// Squared distance from the point to the closest point of the line, with that point and its curve parameter
static float closestOnLine(const Polyline &line, Vector2 point, float &t, Vector2 &closest)
{
    float bestSquared = FLT_MAX;
    for (int i = 0; i + 1 < line.count; ++i)
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

// True when two parts of a curve far apart along it come closer than clearance: it crosses itself, folds back or bends too tight
static bool curveTouchesItself(const Polyline &line, float clearance)
{
    std::array<float, CURVE_POINTS> arc = {};
    for (int i = 1; i < line.count; ++i) arc[i] = arc[i - 1] + Vector2Distance(line.points[i - 1], line.points[i]);
    for (int i = 0; i + 1 < line.count; ++i)
    {
        for (int j = i + 2; j + 1 < line.count; ++j)
        {
            const float distance = segmentsDistance(line.points[i], line.points[i + 1], line.points[j], line.points[j + 1]);
            if (arc[j] - arc[i + 1] >= 2.0f * clearance && distance < clearance) return true;
        }
    }
    return false;
}

// Whether a piece of one curve is nearly parallel to a piece of the other, closer than gap and beside it for more than minShared
static bool curvesRunAlong(const Polyline &first, const Polyline &second, float gap, float minShared)
{
    // Sine of about 12 degrees, below it two pieces count as parallel
    constexpr float MAX_SINE = 0.21f;

    for (int i = 0; i + 1 < first.count; ++i)
    {
        const Vector2 p0 = first.points[i];
        const Vector2 p1 = first.points[i + 1];
        const Vector2 direction = Vector2Normalize(p1 - p0);
        for (int j = 0; j + 1 < second.count; ++j)
        {
            const Vector2 q0 = second.points[j];
            const Vector2 q1 = second.points[j + 1];
            const float crossAt = Vector2CrossProduct(direction, Vector2Normalize(q1 - q0));
            const float distance = segmentsDistance(p0, p1, q0, q1);
            if (fabsf(crossAt) > MAX_SINE || distance >= gap) continue;

            const float a0 = Vector2DotProduct(p0, direction);
            const float a1 = Vector2DotProduct(p1, direction);
            const float b0 = Vector2DotProduct(q0, direction);
            const float b1 = Vector2DotProduct(q1, direction);
            if (fminf(fmaxf(a0, a1), fmaxf(b0, b1)) - fmaxf(fminf(a0, a1), fminf(b0, b1)) > minShared) return true;
        }
    }
    return false;
}

// Appends where the line crosses the line of a road, both count up to touch past their ends so that a line can start on a road
static int lineHits(const Polyline &line, const Polyline &road, int index, float touch, std::span<Hit> hits, int count)
{
    for (int i = 0; i + 1 < line.count; ++i)
    {
        for (int j = 0; j + 1 < road.count && count < (int)hits.size(); ++j)
        {
            float u = 0.0f;
            float v = 0.0f;
            const bool noIntersection = !intersect(line.points[i], line.points[i + 1] - line.points[i], road.points[j], road.points[j + 1] - road.points[j], u, v);
            if (noIntersection || u < -touch || u > 1.0f + touch || v < -touch || v > 1.0f + touch) continue;

            hits[count++] = { ((float)i + u) / (float)(line.count - 1), index, ((float)j + Clamp(v, 0.0f, 1.0f)) / (float)(road.count - 1) };
        }
    }
    return count;
}

static Interval projectedExtent(std::span<const Vector2> points, Vector2 axis)
{
    Interval extent;
    for (Vector2 p : points) extent = { fminf(extent.low, Vector2DotProduct(p, axis)), fmaxf(extent.high, Vector2DotProduct(p, axis)) };
    return extent;
}

static OrientedBox orientedBox(std::span<const Vector2> points)
{
    OrientedBox best;
    float bestArea = FLT_MAX;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector2 side = points[(i + 1) % points.size()] - points[i];
        if (Vector2Length(side) < 0.000001f) continue;

        const Vector2 along = Vector2Normalize(side);
        const Vector2 across = { -along.y, along.x };
        const Interval a = projectedExtent(points, along);
        const Interval b = projectedExtent(points, across);
        const float length = a.high - a.low;
        const float width = b.high - b.low;
        if (length * width >= bestArea) continue;

        bestArea = length * width;
        best = length >= width ? OrientedBox{ along, across, length, width } : OrientedBox{ across, along, width, length };
    }
    return best;
}

// The part of a polygon left of the line, or right of it, cut where the line crosses its sides (Sutherland-Hodgman). The part of a polygon
// that is not convex can come out as loops that touch, which is close enough for its area and box. Returns the corner count, none when out
// has no room
static int clipPolygon(std::span<const Vector2> polygon, const Line &line, bool left, std::span<Vector2> out)
{
    const Vector2 normal = { -line.direction.y, line.direction.x };
    const float sign = left ? 1.0f : -1.0f;
    int count = 0;
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        const Vector2 p = polygon[i];
        const Vector2 q = polygon[(i + 1) % polygon.size()];
        const float sideP = sign * Vector2DotProduct(normal, p - line.origin);
        const float sideQ = sign * Vector2DotProduct(normal, q - line.origin);
        if (count + 2 > (int)out.size()) return 0;

        if (sideP >= 0.0f) out[count++] = p;
        if ((sideP >= 0.0f) != (sideQ >= 0.0f)) out[count++] = Vector2Lerp(p, q, sideP / (sideP - sideQ));
    }
    return count;
}

// ---- Street patterns of the parcel types: the organic, raster and radial morphologies that Shariatpour et al. grow with the streets
// of CityEngine (Computational Urban Science 6, 2026), and random streets. The streets run past the parcel, laying them cuts them to it

// A straight street along the line through point, long enough to cross the polygon
static int addLine(std::span<const Vector2> polygon, Vector2 point, Vector2 direction, std::span<Street> out, int count)
{
    float reach = 0.0f;
    for (Vector2 p : polygon) reach = fmaxf(reach, Vector2Distance(point, p));
    if (count < (int)out.size()) out[count++] = { point - direction * (reach + 1.0f), point + direction * (reach + 1.0f), RoadAttributes() };
    return count;
}

// A street from a to b that meanders like an organic street of CityEngine, each piece turning by up to maxTurn against the one before,
// inside the polygon and back on the straight line at both ends. The pieces follow a Catmull-Rom spline
static int addMeanderingStreet(std::span<const Vector2> polygon, Vector2 a, Vector2 b, float step, float maxTurn, Random &random, std::span<Street> out, int count)
{
    // At most 24 pieces, a parcel between two long streets has to fit into MAX_PARCEL_CORNERS with all samples of their curves
    constexpr int MAX_CORNERS = 25;

    std::array<Vector2, MAX_CORNERS> corners = {};
    const int cornerCount = std::clamp((int)ceilf(Vector2Distance(a, b) / step), 1, MAX_CORNERS - 1) + 1;
    const float piece = Vector2Distance(a, b) / (float)(cornerCount - 1);

    float heading = 0.0f;
    float offset = 0.0f;
    for (int k = 0; k < cornerCount; ++k)
    {
        // The offset from the straight line fades out at both ends and is dropped close to a side
        const float along = (float)k / (float)(cornerCount - 1);
        heading = Clamp(heading + maxTurn * (2.0f * random.Next() - 1.0f), -2.0f * maxTurn, 2.0f * maxTurn);
        offset = Clamp(offset + tanf(heading) * piece, -step, step);
        corners[k] = Vector2Lerp(a, b, along) + leftNormal(a, b) * (offset * sinf(PI * along));

        const bool outsideOrNearSide = !insidePolygon(polygon, corners[k]) || boundaryDistance(polygon, corners[k]) < 0.25f * step;
        if (k > 0 && k + 1 < cornerCount && outsideOrNearSide) corners[k] = Vector2Lerp(a, b, along);
    }
    // The pieces reach from corner k - 1 to k + 2, both kept within the corners there are
    for (int k = 0; k + 1 < cornerCount && k + 1 < (int)corners.size() && count < (int)out.size(); ++k)
    {
        const Vector2 first = corners[k] + (corners[k + 1] - corners[std::max(k - 1, 0)]) / 6.0f;
        const Vector2 second = corners[k + 1] - (corners[std::min(k + 2, cornerCount - 1)] - corners[k]) / 6.0f;
        out[count++] = { corners[k], corners[k + 1], { .curved = true, .controls = { first, second } } };
    }
    return count;
}

// A meandering street over every part of the polygon that the line through point crosses, parts shorter than minLength are left out
static int addMeanderingStreets(std::span<const Vector2> polygon, Vector2 point, Vector2 direction, float step, float minLength, Random &random, std::span<Street> out, int count)
{
    constexpr int MAX_CROSSINGS = 64;

    // Organic streets turn by up to 20 degrees from one piece to the next
    constexpr float MAX_TURN = 0.35f;

    // Where the sides cross the line, in order along it they bound the parts inside in pairs
    const Vector2 normal = { -direction.y, direction.x };
    std::array<float, MAX_CROSSINGS> crossings = {};
    int crossingCount = 0;
    for (size_t i = 0; i < polygon.size() && crossingCount < MAX_CROSSINGS; ++i)
    {
        const Vector2 p = polygon[i];
        const Vector2 q = polygon[(i + 1) % polygon.size()];
        const float sideP = Vector2DotProduct(normal, p - point);
        const float sideQ = Vector2DotProduct(normal, q - point);
        if ((sideP >= 0.0f) == (sideQ >= 0.0f)) continue;

        crossings[crossingCount++] = Vector2DotProduct(direction, Vector2Lerp(p, q, sideP / (sideP - sideQ)) - point);
    }
    std::sort(crossings.begin(), crossings.begin() + crossingCount);
    for (int k = 0; k + 1 < crossingCount; k += 2)
    {
        if (crossings[k + 1] - crossings[k] < minLength) continue;

        count = addMeanderingStreet(polygon, point + direction * crossings[k], point + direction * crossings[k + 1], step, MAX_TURN, random, out, count);
    }
    return count;
}

// Streets along the direction that cut the polygon into strips about spacing wide, moved by up to half the jitter of a strip. With
// meander they meander over the parts of the polygon they cross that are at least minLength long
static int addParallelStreets(std::span<const Vector2> polygon, Vector2 direction, float spacing, float jitter, float meander, float minLength, Random &random, std::span<Street> out, int count)
{
    const Vector2 normal = { -direction.y, direction.x };
    const Interval across = projectedExtent(polygon, normal);
    const Interval along = projectedExtent(polygon, direction);
    const int strips = std::max(1, (int)ceilf((across.high - across.low) / spacing));
    for (int i = 1; i < strips; ++i)
    {
        const float offset = across.low + (across.high - across.low) * ((float)i + jitter * (random.Next() - 0.5f)) / (float)strips;
        const Vector2 point = normal * offset + direction * (0.5f * (along.low + along.high));
        count = meander > 0.0f ? addMeanderingStreets(polygon, point, direction, meander, minLength, random, out, count) : addLine(polygon, point, direction, out, count);
    }
    return count;
}

// Rings around the middle half a block apart, and spokes between them that double further out to keep blocks a block long. Arcs of 30
// degrees stay so close to the circle that spokes that end on it snap to them
static int addRadialStreets(std::span<const Vector2> polygon, Vector2 center, Vector2 axis, float blockSize, std::span<Street> out, int count)
{
    constexpr int ARCS = 12;
    constexpr int FIRST_SPOKES = 4;

    const float spacing = 0.5f * blockSize;
    float reach = 0.0f;
    for (Vector2 p : polygon) reach = fmaxf(reach, Vector2Distance(center, p));
    for (float radius = 0.5f * spacing; radius < reach; radius += spacing)
    {
        // The control point of an arc is where the tangents at its ends meet
        for (int k = 0; k < ARCS && count < (int)out.size(); ++k)
        {
            const Vector2 a = center + Vector2Rotate(axis, 2.0f * PI * (float)k / (float)ARCS) * radius;
            const Vector2 b = center + Vector2Rotate(axis, 2.0f * PI * (float)(k + 1) / (float)ARCS) * radius;
            const Vector2 control = center + Vector2Rotate(axis, 2.0f * PI * ((float)k + 0.5f) / (float)ARCS) * (radius / cosf(PI / (float)ARCS));
            out[count++] = { a, b, bentRoad(a, control, b) };
        }
    }
    int spokes = FIRST_SPOKES;
    for (float inner = 0.5f * spacing; inner < reach; inner += spacing)
    {
        const float outer = inner + spacing;
        while (2.0f * PI * outer / (float)spokes > blockSize) spokes *= 2;
        for (int j = 0; j < spokes && count < (int)out.size(); ++j)
        {
            const Vector2 direction = Vector2Rotate(axis, 2.0f * PI * (float)j / (float)spokes);
            out[count++] = { center + direction * inner, center + direction * outer, RoadAttributes() };
        }
    }
    return count;
}

// Streets for a parcel longer than a block, radial streets go around the middle in its attributes
static int streetPattern(std::span<const Vector2> outline, const ParcelAttributes &attributes, unsigned seed, std::span<Street> out)
{
    // Organic streets meander every fifth of a block, their families cross at changing angles with uneven spacing
    constexpr float ORGANIC_MEANDER = 0.2f;
    constexpr float ORGANIC_ANGLE = 0.6f;
    constexpr float ORGANIC_JITTER = 0.7f;

    const ParcelSettings &settings = attributes.settings;
    const OrientedBox box = orientedBox(outline);
    if (settings.maxBlockSize < MIN_BLOCK_SIZE || box.length <= settings.maxBlockSize) return 0;

    Random random = { seed * 2246822519u | 1u };
    const float longSide = settings.maxBlockSize;
    const float shortSide = 0.5f * settings.maxBlockSize;
    int count = 0;
    switch (attributes.style)
    {
        case SubdivisionStyle_Grid:
        {
            // Raster streets across the parcel a block apart and along it half a block apart
            count = addParallelStreets(outline, box.axisShort, longSide, 0.0f, 0.0f, 0.0f, random, out, count);
            count = addParallelStreets(outline, box.axisLong, shortSide, 0.0f, 0.0f, 0.0f, random, out, count);
            break;
        }
        case SubdivisionStyle_Organic:
        {
            const Vector2 first = Vector2Rotate(box.axisLong, ORGANIC_ANGLE * (random.Next() - 0.5f));
            const Vector2 second = Vector2Rotate(first, 0.5f * PI + ORGANIC_ANGLE * (random.Next() - 0.5f));
            const float meander = ORGANIC_MEANDER * settings.maxBlockSize;
            count = addParallelStreets(outline, first, shortSide, ORGANIC_JITTER, meander, 0.5f * settings.minBlockSize, random, out, count);
            count = addParallelStreets(outline, second, longSide, ORGANIC_JITTER, meander, 0.5f * settings.minBlockSize, random, out, count);
            break;
        }
        case SubdivisionStyle_Radial:
        {
            count = addRadialStreets(outline, attributes.center, box.axisLong, settings.maxBlockSize, out, count);
            break;
        }
        case SubdivisionStyle_Random:
        {
            // Straight streets at any angle somewhere in the middle part, as many as a grid would need. Moved across their direction
            // they spread over the parcel, through one point they would all meet
            const int streets = (int)ceilf(box.length / longSide) + (int)ceilf(box.width / shortSide);
            for (int k = 0; k < streets; ++k)
            {
                const float angle = PI * random.Next();
                const Vector2 direction = { cosf(angle), sinf(angle) };
                const Vector2 normal = { -direction.y, direction.x };
                const Interval along = projectedExtent(outline, direction);
                const Interval across = projectedExtent(outline, normal);
                const Vector2 point = direction * (0.5f * (along.low + along.high)) + normal * Lerp(across.low, across.high, 0.2f + 0.6f * random.Next());
                count = addLine(outline, point, direction, out, count);
            }
            break;
        }
        default: break;
    }
    return count;
}

// The smallest box around the points of both lists, the oriented box of their convex hull. Quicker than orientedBox for the many corners
// of a block along curved streets, and around two blocks it is the box of the block they make when they are merged
static OrientedBox hullBox(std::span<const Vector2> first, std::span<const Vector2> second)
{
    std::array<Vector2, 2 * MAX_PARCEL_CORNERS> sorted;
    std::array<Vector2, 2 * MAX_PARCEL_CORNERS> hull;

    const size_t firstCount = std::min(first.size(), sorted.size());
    const size_t n = firstCount + std::min(second.size(), sorted.size() - firstCount);

    std::copy(first.begin(), first.begin() + (long)firstCount, sorted.begin());
    std::copy(second.begin(), second.begin() + (long)(n - firstCount), sorted.begin() + (long)firstCount);
    std::sort(sorted.begin(), sorted.begin() + (long)n, [](Vector2 a, Vector2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; });
    if (n < 3) return orientedBox({ sorted.data(), n });

    // Andrew's monotone chain, the lower half from left to right and the upper half back, counter-clockwise
    int count = 0;
    for (size_t i = 0; i < n; ++i)
    {
        while (count >= 2 && Vector2CrossProduct(hull[count - 1] - hull[count - 2], sorted[i] - hull[count - 2]) <= 0.0f) --count;
        hull[count++] = sorted[i];
    }

    const int lower = count + 1;
    for (size_t i = n - 1; i-- > 0;)
    {
        while (count >= lower && Vector2CrossProduct(hull[count - 1] - hull[count - 2], sorted[i] - hull[count - 2]) <= 0.0f) --count;
        hull[count++] = sorted[i];
    }

    // The last corner is the first one again. Points about on one line make no hull to go round
    const int h = count - 1;
    if (h < 3 || fabsf(signedArea({ hull.data(), (size_t)h })) < 0.01f) return orientedBox({ sorted.data(), n });

    // Rotating calipers: the box along every side of the hull, with the corners farthest ahead, inside and behind it. Those only move on
    // as the sides turn, which makes it linear. They pass corners that are as far, of a side square to the direction or two corners at
    // about one place, at most once around
    OrientedBox best;
    float bestArea = FLT_MAX;
    int ahead = 1;
    int inside = 1;
    int behind = 1;
    for (int i = 0; i < h; ++i)
    {
        const Vector2 along = Vector2Normalize(hull[i + 1] - hull[i]);
        const Vector2 across = { -along.y, along.x };
        for (int step = 0; step < h && Vector2DotProduct(hull[(ahead + 1) % h], along) >= Vector2DotProduct(hull[ahead], along); ++step) ahead = (ahead + 1) % h;
        if (i == 0) inside = ahead;
        for (int step = 0; step < h && Vector2DotProduct(hull[(inside + 1) % h], across) >= Vector2DotProduct(hull[inside], across); ++step) inside = (inside + 1) % h;
        if (i == 0) behind = inside;
        for (int step = 0; step < h && Vector2DotProduct(hull[(behind + 1) % h], along) <= Vector2DotProduct(hull[behind], along); ++step) behind = (behind + 1) % h;

        const float length = Vector2DotProduct(hull[ahead] - hull[behind], along);
        const float width = Vector2DotProduct(hull[inside] - hull[i], across);
        if (length * width >= bestArea) continue;

        bestArea = length * width;
        best = length >= width ? OrientedBox{ along, across, length, width } : OrientedBox{ across, along, width, length };
    }
    return best;
}

// Lines across a block longer than a block, at a few places around the middle of its long side. They are square to the long side, out of
// the middle of a radial pattern where that runs across the block, and turned a little at random for organic and random blocks. A line
// counts when both parts reach less far along the long side than the block or no farther than a block may, and are at least as wide as
// minBlockSize, or as three quarters of a block that narrow itself, a hundredth less too, since organic streets meander exactly as far as
// a part can be wide. The line that leaves the narrower part widest comes first. Returns the line count
static int blockSplits(std::span<const Vector2> outline, const OrientedBox &box, const ParcelAttributes &attributes, Random &random, std::span<Line> lines)
{
    constexpr int PLACES = 9;
    constexpr float MAX_TURN = 0.3f;
    constexpr float TOLERANCE = 0.99f;

    // Cosine of 45 degrees, a line out of the middle that runs closer along the long side would cut the block along it. A part has to
    // reach less far along the long side by this much
    constexpr float ACROSS = 0.71f;
    constexpr float SHORTER = 0.85f;

    std::array<Vector2, 2 * MAX_PARCEL_CORNERS> part;
    std::array<float, PLACES> widths = {};
    const Interval along = projectedExtent(outline, box.axisLong);
    const Interval across = projectedExtent(outline, box.axisShort);
    const bool turns = attributes.style == SubdivisionStyle_Organic || attributes.style == SubdivisionStyle_Random;
    const float minWidth = TOLERANCE * fminf(attributes.settings.minBlockSize, 0.75f * box.width);
    int count = 0;
    for (int k = 0; k < PLACES && count < (int)lines.size(); ++k)
    {
        // From the middle out, a twentieth of the length further every second place
        const float fraction = 0.5f + 0.05f * (float)((k + 1) / 2) * (k % 2 == 0 ? 1.0f : -1.0f);
        const Vector2 point = box.axisLong * Lerp(along.low, along.high, fraction) + box.axisShort * (0.5f * (across.low + across.high));
        Vector2 direction = box.axisShort;
        const Vector2 out = Vector2Normalize(point - attributes.center);
        if (attributes.style == SubdivisionStyle_Radial && Vector2Distance(point, attributes.center) > 1.0f && fabsf(Vector2DotProduct(out, box.axisLong)) < ACROSS) direction = out;
        if (turns) direction = Vector2Rotate(direction, MAX_TURN * (2.0f * random.Next() - 1.0f));

        const Line line = { point, direction };
        float width = FLT_MAX;
        for (bool left : { true, false })
        {
            const std::span<const Vector2> piece = { part.data(), (size_t)clipPolygon(outline, line, left, part) };
            const Interval reach = projectedExtent(piece, box.axisLong);
            const bool shorter = reach.high - reach.low <= fmaxf(SHORTER * box.length, attributes.settings.maxBlockSize);
            width = piece.size() < 3 || !shorter ? 0.0f : fminf(width, hullBox(piece, {}).width);
        }
        if (width < minWidth) continue;

        // Into its place by width
        int at = count++;
        for (; at > 0 && widths[at - 1] < width; --at)
        {
            widths[at] = widths[at - 1];
            lines[at] = lines[at - 1];
        }
        widths[at] = width;
        lines[at] = line;
    }
    return count;
}

// ---- Lots after Chen, Song and Ortner, Hierarchical Co-generation of Parcels and Streets in Urban Modeling (CGF 2024), and Vanegas
// et al., Procedural Generation of Parcels in Urban Modeling (CGF 2012). A block is split in two along the best candidate line until
// its lots are small enough, every lot keeps a side along a road

static std::span<const Vector2> corners(const Cell &cell)
{
    return { cell.points.data(), (size_t)cell.count };
}

// The length of the sides of the cell that run along a road
static float frontage(const Cell &cell)
{
    float length = 0.0f;
    for (int i = 0; i < cell.count; ++i)
    {
        if (cell.street[i]) length += Vector2Distance(cell.points[i], cell.points[(i + 1) % cell.count]);
    }
    return length;
}

// The least road that any of count equal pieces of the cell gets when it is cut across the axis, each side spread over the pieces it spans
static float leastRoad(const Cell &cell, Vector2 axis, int count)
{
    constexpr int MAX_PIECES = 16;

    std::array<float, MAX_PIECES> road = {};
    count = std::clamp(count, 1, MAX_PIECES);
    const Interval extent = projectedExtent(corners(cell), axis);
    const float step = (extent.high - extent.low) / (float)count;
    if (step <= 0.0f) return 0.0f;

    for (int i = 0; i < cell.count; ++i)
    {
        if (!cell.street[i]) continue;

        // Where the side starts and ends in pieces, a side square to the axis is all in one
        const Vector2 a = cell.points[i];
        const Vector2 b = cell.points[(i + 1) % cell.count];
        const float length = Vector2Distance(a, b);
        const float from = fminf(Vector2DotProduct(a, axis), Vector2DotProduct(b, axis)) - extent.low;
        const float to = fmaxf(Vector2DotProduct(a, axis), Vector2DotProduct(b, axis)) - extent.low;
        if (to - from < 0.001f)
        {
            road[std::clamp((int)(from / step), 0, count - 1)] += length;
            continue;
        }
        for (int k = std::clamp((int)(from / step), 0, count - 1); k < count && (float)k * step < to; ++k) road[k] += length * (fminf(to, (float)(k + 1) * step) - fmaxf(from, (float)k * step)) / (to - from);
    }
    return *std::min_element(road.begin(), road.begin() + count);
}

// Cuts off the loop where two sides of the cell cross or two corners meet, the larger part stays. The roads inset at a sharp angle
// between two roads cross a little
static void untangle(Cell &cell)
{
    std::array<Cell, 2> loops;
    for (bool tangled = true; tangled;)
    {
        tangled = false;
        const int n = cell.count;
        for (int i = 0; i < n && !tangled; ++i)
        {
            for (int j = i + 1; j < n && !tangled; ++j)
            {
                // Two corners at one place start both loops, two sides that cross start them at the crossing
                float u = 0.0f;
                float v = 0.0f;
                const bool same = Vector2Distance(cell.points[i], cell.points[j]) < 0.001f;
                const bool notCrossing = !segmentsCross(cell.points[i], cell.points[i + 1], cell.points[j], cell.points[(j + 1) % n]);
                if (!same && (j == i + 1 || (i == 0 && j + 1 == n) || notCrossing)) continue;

                loops[0].count = 0;
                loops[1].count = 0;
                if (!same && intersect(cell.points[i], cell.points[i + 1] - cell.points[i], cell.points[j], cell.points[(j + 1) % n] - cell.points[j], u, v))
                {
                    loops[0].Add(Vector2Lerp(cell.points[i], cell.points[i + 1], u), true);
                    loops[1].Add(loops[0].points[0], true);
                }
                for (int k = i + 1; k <= j; ++k) loops[0].Add(cell.points[k], true);
                for (int k = j + 1; k <= n + i; ++k) loops[1].Add(cell.points[k % n], true);
                cell = fabsf(signedArea(corners(loops[0]))) > fabsf(signedArea(corners(loops[1]))) ? loops[0] : loops[1];
                tangled = true;
            }
        }
    }
}

// The outline of a parcel as a cell, every side along a road. Corners that the line past them misses by less than the tolerance, like
// most samples of a gentle curve, only make splitting slower. When that makes the two sides of the slit of a curved dead end cross, the
// tolerance drops below half the width of a slit
static bool thinOutline(std::span<const Vector2> outline, Cell &cell)
{
    const size_t n = outline.size();
    for (float tolerance : { 0.05f, 0.01f })
    {
        cell.count = 0;
        for (size_t i = 0, from = 0; i < n; ++i)
        {
            bool straight = i > 0;
            for (size_t k = from + 1; k <= i && straight; ++k) straight = pointSegmentDistance(outline[k], outline[from], outline[(i + 1) % n]) <= tolerance;
            if (straight) continue;
            if (!cell.Add(outline[i], true)) return false;

            from = i;
        }
        if (cell.count > 1 && Vector2Distance(cell.points[0], cell.points[cell.count - 1]) < 0.001f) --cell.count;
        if (polygonIsSimple(corners(cell))) return true;
    }
    untangle(cell);
    return polygonIsSimple(corners(cell));
}

// Parallel lines spread from low to high over the width of the polygon across the direction, while there is room for them
static int addParallelLines(std::span<const Vector2> points, Vector2 direction, int lineCount, float low, float high, std::span<Line> lines, int count)
{
    const Vector2 normal = { -direction.y, direction.x };
    const Interval across = projectedExtent(points, normal);
    for (int k = 0; k < lineCount && count < (int)lines.size(); ++k)
    {
        const float fraction = lineCount > 1 ? low + (high - low) * (float)k / (float)(lineCount - 1) : low;
        lines[count++] = { normal * (across.low + fraction * (across.high - across.low)), direction };
    }
    return count;
}

// Candidates for the line of a split, spread over the middle of the cell
static int candidateLines(std::span<const Vector2> points, const OrientedBox &box, const ParcelAttributes &attributes, Random &random, std::span<Line> lines)
{
    constexpr float TURN = 0.26f;

    int count = 0;
    switch (attributes.style)
    {
        case SubdivisionStyle_Grid:
        {
            // Lines across and along the cell, a regular grid of lots
            count = addParallelLines(points, box.axisShort, 9, 0.25f, 0.75f, lines, count);
            count = addParallelLines(points, box.axisLong, 9, 0.25f, 0.75f, lines, count);
            break;
        }
        case SubdivisionStyle_Organic:
        {
            // Slightly turned lines, the lots adapt to the shape
            for (int turn = -1; turn <= 1; ++turn)
            {
                count = addParallelLines(points, Vector2Rotate(box.axisShort, (float)turn * TURN), 3, 0.35f, 0.65f, lines, count);
                count = addParallelLines(points, Vector2Rotate(box.axisLong, (float)turn * TURN), 3, 0.35f, 0.65f, lines, count);
            }
            break;
        }
        case SubdivisionStyle_Radial:
        {
            // Lines out of the middle over the angle of the cell and lines around it make wedges, a cell around the middle is cut through it
            const Vector2 center = attributes.center;
            const Vector2 outward = Vector2Normalize(interiorPoint(points) - center);
            if (insidePolygon(points, center) || Vector2Length(outward) < 0.5f)
            {
                for (int k = 0; k < 9; ++k) lines[count++] = { center, Vector2Rotate(box.axisLong, PI * (float)k / 9.0f) };
                break;
            }

            Interval angles = { 0.0f, 0.0f };
            for (Vector2 p : points) angles = { fminf(angles.low, Vector2Angle(outward, p - center)), fmaxf(angles.high, Vector2Angle(outward, p - center)) };
            for (int k = 0; k < 9; ++k) lines[count++] = { center, Vector2Rotate(outward, Lerp(angles.low, angles.high, 0.25f + 0.0625f * (float)k)) };
            count = addParallelLines(points, { -outward.y, outward.x }, 9, 0.25f, 0.75f, lines, count);
            break;
        }
        case SubdivisionStyle_Random:
        {
            while (count < (int)lines.size())
            {
                const float angle = PI * random.Next();
                count = addParallelLines(points, { cosf(angle), sinf(angle) }, 1, 0.3f + 0.4f * random.Next(), 0.0f, lines, count);
            }
            break;
        }
        default: break;
    }
    return count;
}

// Lines through points spread over the road along the cell, square to the road there and along both sides of the oriented box of the cell.
// A split along one of them leaves road on both parts. Appends to the count lines there are
static int accessLines(const Cell &cell, const OrientedBox &box, std::span<Line> lines, int count)
{
    // The road is cut into twelfths, so its halves, thirds, quarters and sixths are all among the points. A line along a side of the box
    // that runs about along the road would not leave it
    constexpr int PIECES = 12;
    constexpr float ALONG_ROAD = 0.9f;

    const float total = frontage(cell);
    for (int k = 1; k < PIECES; ++k)
    {
        float along = total * (float)k / (float)PIECES;
        for (int i = 0; i < cell.count; ++i)
        {
            const Vector2 a = cell.points[i];
            const Vector2 b = cell.points[(i + 1) % cell.count];
            const float length = Vector2Distance(a, b);
            if (!cell.street[i] || length <= 0.0f) continue;
            if (along > length)
            {
                along -= length;
                continue;
            }

            const Vector2 point = Vector2Lerp(a, b, along / length);
            const Vector2 road = (b - a) / length;
            if (count < (int)lines.size()) lines[count++] = { point, leftNormal(a, b) };
            for (Vector2 axis : { box.axisLong, box.axisShort })
            {
                if (count < (int)lines.size() && fabsf(Vector2DotProduct(axis, road)) < ALONG_ROAD) lines[count++] = { point, axis };
            }
            break;
        }
    }
    return count;
}

// Lines along both sides of the oriented box of the cell that cut it into parts of a half and a third of its area. Appends to the count
// lines there are
static int areaLines(const Cell &cell, const OrientedBox &box, std::span<Line> lines, int count)
{
    constexpr int STEPS = 24;

    std::array<Vector2, 2 * MAX_CELL_POINTS> part;
    const float total = signedArea(corners(cell));
    for (Vector2 direction : { box.axisShort, box.axisLong })
    {
        // The area left of the line grows as the line moves to its right, the place is found by halving the extent across it
        const Vector2 normal = { -direction.y, direction.x };
        const Interval extent = projectedExtent(corners(cell), normal);
        for (float share : { 0.5f, 1.0f / 3.0f, 2.0f / 3.0f })
        {
            Interval range = extent;
            for (int step = 0; step < STEPS; ++step)
            {
                const float middle = 0.5f * (range.low + range.high);
                const int partCount = clipPolygon(corners(cell), { normal * middle, direction }, true, part);
                if (signedArea({ part.data(), (size_t)partCount }) > share * total) range.low = middle;
                else range.high = middle;
            }
            if (count < (int)lines.size()) lines[count++] = { normal * (0.5f * (range.low + range.high)), direction };
        }
    }
    return count;
}

// The crossings of the sides of the cell with the line in order along it, in pairs they bound the parts of the line inside. A crossing
// close to a corner moves onto it, so that splits leave no slivers
static int lineCrossings(const Cell &cell, const Line &line, std::span<Crossing> out)
{
    constexpr float SNAP_DISTANCE = 0.1f;

    const Vector2 normal = { -line.direction.y, line.direction.x };
    int count = 0;
    for (int i = 0; i < cell.count && count < (int)out.size(); ++i)
    {
        const int next = (i + 1) % cell.count;
        const float sideA = Vector2DotProduct(normal, cell.points[i] - line.origin);
        const float sideB = Vector2DotProduct(normal, cell.points[next] - line.origin);
        if ((sideA >= 0.0f) == (sideB >= 0.0f)) continue;

        Crossing crossing = { i, Vector2Lerp(cell.points[i], cell.points[next], sideA / (sideA - sideB)), 0.0f };
        if (Vector2Distance(crossing.point, cell.points[i]) < SNAP_DISTANCE) crossing.point = cell.points[i];
        else if (Vector2Distance(crossing.point, cell.points[next]) < SNAP_DISTANCE) crossing = { next, cell.points[next], 0.0f };
        crossing.t = Vector2DotProduct(line.direction, crossing.point - line.origin);
        out[count++] = crossing;
    }
    std::sort(out.begin(), out.begin() + count, [](const Crossing &first, const Crossing &second) { return first.t < second.t; });
    return count;
}

// Whether a crossing is on a road, one on a corner also when the side before the corner is
static bool onStreet(const Cell &cell, const Crossing &crossing)
{
    const int before = (crossing.side + cell.count - 1) % cell.count;
    return cell.street[crossing.side] || (Vector2Distance(crossing.point, cell.points[crossing.side]) < 0.001f && cell.street[before]);
}

// The chord from a to b bent into a gentle curve that stays inside the polygon, its corners between a and b and its road. False when
// neither way of bending fits
static bool bendChord(std::span<const Vector2> polygon, Vector2 a, Vector2 b, Random &random, std::array<Vector2, BEND_POINTS> &bend, RoadAttributes &road)
{
    // The most the chord bends in parts of its length, the length below which it stays straight, the room kept from the sides
    constexpr float MAX_BEND = 0.2f;
    constexpr float MIN_LENGTH = 12.0f;
    constexpr float CLEARANCE = 0.3f;

    if (Vector2Distance(a, b) < MIN_LENGTH) return false;

    const float sign = random.Next() < 0.5f ? 1.0f : -1.0f;
    const float amount = 2.0f * MAX_BEND * (0.3f + 0.7f * random.Next());
    const Vector2 side = Vector2{ a.y - b.y, b.x - a.x } * (sign * amount);
    for (float flip : { 1.0f, -1.0f })
    {
        road = bentRoad(a, Vector2Lerp(a, b, 0.5f) + side * flip, b);
        const Bezier curve = roadBezier(a, road, b);
        bool fits = true;
        for (int k = 0; k < BEND_POINTS && fits; ++k)
        {
            bend[k] = bezierPoint(curve, (float)(k + 1) / (float)(BEND_POINTS + 1));
            fits = insidePolygon(polygon, bend[k]) && boundaryDistance(polygon, bend[k]) > CLEARANCE;
        }
        for (int k = 0; k <= BEND_POINTS && fits; ++k)
        {
            const Vector2 from = k > 0 ? bend[k - 1] : a;
            const Vector2 to = k < BEND_POINTS ? bend[k] : b;
            for (size_t i = 0; i < polygon.size() && fits; ++i) fits = !segmentsCross(from, to, polygon[i], polygon[(i + 1) % polygon.size()]);
        }
        if (fits) return true;
    }
    road = RoadAttributes();
    return false;
}

// The parts of the cell on both sides of the chord between two crossings. A bent chord has its corners from a to b in bend, street tells
// whether the chord runs along a road
static bool cut(const Cell &cell, Crossing a, Crossing b, std::span<const Vector2> bend, bool street, std::array<Cell, 2> &parts)
{
    if (a.side == b.side) return false;
    if (a.side > b.side) std::swap(a, b);

    // The first part runs from a along the sides to b and back along the chord, the second from b along the other sides to a
    const int n = cell.count;
    parts[0].count = 0;
    parts[1].count = 0;

    bool fits = parts[0].Add(a.point, cell.street[a.side]);
    for (int k = a.side + 1; k <= b.side && fits; ++k) fits = parts[0].Add(cell.points[k], cell.street[k]);

    fits = fits && parts[0].Add(b.point, street);
    for (size_t k = bend.size(); k-- > 0 && fits;) fits = parts[0].Add(bend[k], street);

    fits = fits && parts[1].Add(b.point, cell.street[b.side]);
    for (int k = b.side + 1; k <= n + a.side && fits; ++k) fits = parts[1].Add(cell.points[k % n], cell.street[k % n]);

    fits = fits && parts[1].Add(a.point, street);
    for (size_t k = 0; k < bend.size() && fits; ++k) fits = parts[1].Add(bend[k], street);

    // A chord that ends on a corner repeats it, the side that leaves the corner stays
    for (Cell &part : parts)
    {
        if (part.count > 1 && Vector2Distance(part.points[0], part.points[part.count - 1]) < 0.001f) --part.count;
    }
    return fits;
}

// How good a split is by Equation 2 of Chen et al.: parts alike in size, regular, and reached from a road. The regularity of a part is
// how much of its oriented box it fills. -FLT_MAX for a split that leaves a part too small, too narrow or without a road along it as long
// as a lot is wide, counting the street of length streetLength the split lays. With budget a part needs that much road for every lot it
// still has to give, so that it can be split on within the settings. Like Chen et al. a street is rated without the road it brings
static float splitQuality(const std::array<Cell, 2> &parts, const ParcelSettings &settings, float streetLength, bool budget)
{
    constexpr float SIZE_WEIGHT = 0.3f;
    constexpr float REGULARITY_WEIGHT = 0.5f;
    constexpr float ACCESS_WEIGHT = 0.2f;

    std::array<float, 2> area = {};
    std::array<float, 2> street = {};
    for (int k = 0; k < 2; ++k)
    {
        // The fewest lots the part can become by its area, the box is only needed for parts that pass
        street[k] = frontage(parts[k]);
        area[k] = signedArea(corners(parts[k]));
        const float lots = budget ? ceilf(area[k] / fmaxf(settings.maxParcelArea, 0.001f)) : 1.0f;
        if (parts[k].count < 3 || area[k] < settings.minParcelArea || street[k] + streetLength < settings.minParcelWidth * lots) return -FLT_MAX;
    }

    float regularity = 0.0f;
    bool reached = true;
    for (int k = 0; k < 2; ++k)
    {
        // The fewest lots the part can become by its area and its width, which have to fit into its area. A part too narrow to be split
        // along its length is cut across into them, and every piece needs road
        const OrientedBox box = orientedBox(corners(parts[k]));
        const float lots = budget ? fmaxf(ceilf(area[k] / fmaxf(settings.maxParcelArea, 0.001f)), ceilf(box.width / fmaxf(settings.maxParcelWidth, 0.001f))) : 1.0f;
        const bool lotsFit = !budget || lots * settings.minParcelArea <= area[k];
        const bool reaches = lots < 2.0f || streetLength > 0.0f || box.width >= 2.0f * settings.minParcelWidth || leastRoad(parts[k], box.axisLong, (int)lots) >= settings.minParcelWidth;
        if (box.width < settings.minParcelWidth || street[k] + streetLength < settings.minParcelWidth * lots || !lotsFit || !reaches) return -FLT_MAX;

        // A part counts as reached when its road is at least half its mean side long
        regularity += 0.5f * area[k] / fmaxf(box.length * box.width, 0.000001f);
        reached = reached && street[k] >= 0.25f * (box.length + box.width);
    }
    return SIZE_WEIGHT * fminf(area[0], area[1]) / fmaxf(area[0], area[1]) + REGULARITY_WEIGHT * regularity + ACCESS_WEIGHT * (reached ? 2.0f : 1.0f);
}

// The best chord of the lines by the rule, -FLT_MAX for none, with its ends and whether it lays a street. With streets an organic chord
// becomes a street where the type wants one, by SplitRule_Street a chord of any type does. A street starts on a road
static float bestChord(const Cell &cell, const OrientedBox &box, const ParcelAttributes &attributes, std::span<const Line> lines, SplitRule rule, bool streets, Random &random, std::array<Crossing, 2> &ends, bool &street)
{
    // How much the irregularity setting can change the quality of a split
    constexpr float NOISE_SCALE = 0.5f;

    std::array<Crossing, MAX_CELL_POINTS> crossings;
    std::array<Cell, 2> candidate;

    const ParcelSettings &settings = attributes.settings;
    const bool organic = attributes.style == SubdivisionStyle_Organic;
    const bool wide = box.width > 0.5f * settings.maxBlockSize;
    float best = -FLT_MAX;
    for (const Line &line : lines)
    {
        const int crossingCount = lineCrossings(cell, line, crossings);
        for (int k = 0; k + 1 < crossingCount; k += 2)
        {
            Crossing a = crossings[k];
            Crossing b = crossings[k + 1];
            if (a.side > b.side) std::swap(a, b);
            if (!cut(cell, a, b, {}, false, candidate)) continue;

            const float length = Vector2Distance(a.point, b.point);
            const bool organicStreet = organic && rule == SplitRule_Settings && (wide || frontage(candidate[0]) < settings.minParcelWidth || frontage(candidate[1]) < settings.minParcelWidth);
            const bool isStreet = streets && (organicStreet || rule == SplitRule_Street);
            if (isStreet && ((!onStreet(cell, a) && !onStreet(cell, b)) || length < settings.minParcelWidth)) continue;

            float quality = splitQuality(candidate, settings, isStreet ? length : 0.0f, rule != SplitRule_Lenient);
            if (quality == -FLT_MAX) continue;

            quality += settings.splitIrregularity * NOISE_SCALE * (2.0f * random.Next() - 1.0f);
            if (quality <= best) continue;

            best = quality;
            street = isStreet;
            ends = { a, b };
        }
    }
    return best;
}

// Splits the cell along the best chord of the candidate lines. Organic chords bend and the parts follow them. With streets an organic
// chord becomes a street, the I-shaped street of Chen et al, when the cell is wider than half a block or a part would have less road
// than a lot is wide. Such a chord starts on a road and is given in chord. When no line of the type splits the cell within the settings,
// lines square to its road follow, then with streets a chord of any type becomes a street, and last a split that only keeps road on both
// parts is taken
static bool splitCell(const Cell &cell, const ParcelAttributes &attributes, bool streets, Random &random, std::array<Cell, 2> &parts, bool &street, Street &chord)
{
    constexpr int MAX_LINES = 20;
    constexpr int MAX_FALLBACK_LINES = 40;

    std::array<Line, MAX_LINES + MAX_FALLBACK_LINES> lines;
    std::array<Vector2, BEND_POINTS> bend;
    std::array<Crossing, 2> ends;

    const ParcelSettings &settings = attributes.settings;
    const OrientedBox box = orientedBox(corners(cell));
    const bool organic = attributes.style == SubdivisionStyle_Organic;

    // The lines of the type first. The lines that keep road on both parts and the lines of equal area are only made when those fail, then
    // with streets a street of any type is tried, and last a split that only keeps road on both parts
    const int typeCount = candidateLines(corners(cell), box, attributes, random, { lines.data(), (size_t)MAX_LINES });
    street = false;
    SplitRule rule = SplitRule_Settings;
    float best = bestChord(cell, box, attributes, { lines.data(), (size_t)typeCount }, rule, streets, random, ends, street);
    if (best == -FLT_MAX)
    {
        const int lineCount = areaLines(cell, box, lines, accessLines(cell, box, lines, typeCount));
        const std::span<const Line> all = { lines.data(), (size_t)lineCount };
        best = bestChord(cell, box, attributes, all.subspan((size_t)typeCount), rule, streets, random, ends, street);
        for (SplitRule fallback : { SplitRule_Street, SplitRule_Lenient })
        {
            if (best != -FLT_MAX || (fallback == SplitRule_Street && !streets)) continue;

            rule = fallback;
            best = bestChord(cell, box, attributes, all, rule, streets, random, ends, street);
        }
    }
    if (best == -FLT_MAX) return false;

    // An organic chord bends when the parts still fit, the parts along a street follow every sample of its road to end where it runs
    chord = { ends[0].point, ends[1].point, RoadAttributes() };
    if (organic && bendChord(corners(cell), chord.a, chord.b, random, bend, chord.road))
    {
        const Polyline line = roadLine(chord.a, chord.road, chord.b);
        const std::span<const Vector2> along = street ? std::span<const Vector2>(line.points.data() + 1, (size_t)line.count - 2) : std::span<const Vector2>(bend);
        if (cut(cell, ends[0], ends[1], along, street, parts) && splitQuality(parts, settings, 0.0f, rule != SplitRule_Lenient) > -FLT_MAX) return true;
    }

    chord.road = RoadAttributes();
    return cut(cell, ends[0], ends[1], {}, street, parts);
}

// ---- Houses

static bool isDisk(const Prop &prop)
{
    return prop.kind == PropKind_Tree || prop.kind == PropKind_Bush;
}

// Whether the next side turns away from the side by less than about 40 degrees, false for a side the polygon does not have
static bool goesStraightOn(std::span<const Vector2> polygon, int side)
{
    const int n = (int)polygon.size();
    if (n < 3 || side < 0 || side >= n) return false;

    const Vector2 first = Vector2Normalize(polygon[(side + 1) % n] - polygon[side]);
    return Vector2DotProduct(first, Vector2Normalize(polygon[(side + 2) % n] - polygon[(side + 1) % n])) > 0.75f;
}

// The corners of a box turned like the axis
static std::array<Vector2, 4> boxCorners(Vector2 center, Vector2 axis, Vector2 halfSize)
{
    const Vector2 along = axis * halfSize.x;
    const Vector2 across = Vector2{ -axis.y, axis.x } * halfSize.y;
    return { center - along - across, center + along - across, center + along + across, center - along + across };
}

// How far apart two props are, zero when they overlap. All boxes are turned like the axis, so the boxes of both add up to one box around
// the middle of the other prop
static float propDistance(const Prop &first, const Prop &second, Vector2 axis)
{
    const Vector2 offset = { fabsf(Vector2DotProduct(first.position - second.position, axis)), fabsf(Vector2CrossProduct(axis, first.position - second.position)) };
    const Vector2 halfSizes = (isDisk(first) ? Vector2Zero() : first.halfSize) + (isDisk(second) ? Vector2Zero() : second.halfSize);
    const float radii = (isDisk(first) ? first.halfSize.x : 0.0f) + (isDisk(second) ? second.halfSize.x : 0.0f);
    return fmaxf(Vector2Length(Vector2Max(offset - halfSizes, Vector2Zero())) - radii, 0.0f);
}

// Whether the prop keeps its room from the sides of the lot, from the roads and from what was placed before
static bool propFits(const Ground &ground, Vector2 axis, const Prop &prop, std::span<const Prop> placed)
{
    constexpr float GAP = 0.4f;

    if (!insidePolygon(ground.lot, prop.position)) return false;

    const std::span<const Vector2> lot = ground.lot;
    const std::array<Vector2, 4> corners = boxCorners(prop.position, axis, prop.halfSize);
    for (size_t i = 0; i < lot.size(); ++i)
    {
        const Vector2 a = lot[i];
        const Vector2 b = lot[(i + 1) % lot.size()];
        const float clearance = ground.streets[i] ? STREET_CLEARANCE : SIDE_CLEARANCE;
        if (isDisk(prop) && pointSegmentDistance(prop.position, a, b) < prop.halfSize.x + clearance) return false;
        for (int k = 0; k < 4 && !isDisk(prop); ++k)
        {
            if (segmentsDistance(a, b, corners[k], corners[(k + 1) % 4]) < clearance) return false;
        }
    }
    const Box reach = inflate(isDisk(prop) ? Box{ prop.position, prop.position } : boxOf(corners), (isDisk(prop) ? prop.halfSize.x : 0.0f) + STREET_CLEARANCE);
    for (const Road &road : ground.roads)
    {
        for (int i = 0; i + 1 < road.line.count && overlap(reach, road.box); ++i)
        {
            const Vector2 a = road.line.points[i];
            const Vector2 b = road.line.points[i + 1];
            if (isDisk(prop) && pointSegmentDistance(prop.position, a, b) < prop.halfSize.x + STREET_CLEARANCE) return false;
            for (int k = 0; k < 4 && !isDisk(prop); ++k)
            {
                if (segmentsDistance(a, b, corners[k], corners[(k + 1) % 4]) < STREET_CLEARANCE) return false;
            }
        }
    }
    for (const Prop &other : placed)
    {
        if (propDistance(prop, other, axis) < GAP) return false;
    }
    return true;
}

// Tries spots in the box of the lot at random until the prop fits there
static void placeProp(const Ground &ground, Box box, Vector2 axis, Prop prop, Random &random, std::span<Prop> placed, int &count)
{
    constexpr int ATTEMPTS = 12;

    for (int attempt = 0; attempt < ATTEMPTS && count < (int)placed.size(); ++attempt)
    {
        prop.position = { Lerp(box.min.x, box.max.x, random.Next()), Lerp(box.min.y, box.max.y, random.Next()) };
        if (!propFits(ground, axis, prop, placed.first((size_t)count))) continue;

        placed[count++] = prop;
        return;
    }
}

// Trees and bushes where there is room, the garden of a lot that has no room for a house
static int plantGarden(const Ground &ground, Box box, Random &random, std::span<Prop> placed, int count)
{
    for (int k = 2 + (int)(4.0f * random.Next()); k > 0; --k) placeProp(ground, box, { 1.0f, 0.0f }, { PropKind_Tree, {}, { Lerp(1.2f, 2.2f, random.Next()), 0.0f }, 0 }, random, placed, count);
    for (int k = 2 + (int)(3.0f * random.Next()); k > 0; --k) placeProp(ground, box, { 1.0f, 0.0f }, { PropKind_Bush, {}, { Lerp(0.5f, 0.8f, random.Next()), 0.0f }, 0 }, random, placed, count);
    return count;
}

static void drawBox(Vector2 center, Vector2 axis, Vector2 halfSize, Color color)
{
    DrawRectanglePro({ center.x, center.y, 2.0f * halfSize.x, 2.0f * halfSize.y }, halfSize, RAD2DEG * atan2f(axis.y, axis.x), color);
}

// The house with a gable roof along its longer side, the ground below it first and the tree tops above it last. A lot without a house
// shows its garden
static void drawHouse(const House &house, float worldPerPixel)
{
    constexpr Color ROOF_COLORS[] = { { 150, 88, 72, 255 }, { 104, 112, 128, 255 }, { 168, 146, 110, 255 }, { 122, 74, 68, 255 }, { 92, 104, 92, 255 } };
    constexpr Color CAR_COLORS[] = { { 176, 66, 62, 255 }, { 222, 224, 230, 255 }, { 66, 106, 168, 255 }, { 46, 48, 54, 255 } };
    constexpr Color EDGE_COLOR = { 30, 32, 38, 255 };
    constexpr Color DRIVEWAY_COLOR = { 106, 110, 120, 255 };
    constexpr Color GLASS_COLOR = { 40, 44, 54, 255 };
    constexpr Color POOL_COLOR = { 198, 206, 216, 255 };
    constexpr Color WATER_COLOR = { 82, 158, 198, 255 };
    constexpr Color SHED_COLOR = { 136, 116, 92, 255 };
    constexpr Color TREE_COLOR = { 46, 86, 60, 255 };
    constexpr Color LEAVES_COLOR = { 68, 116, 78, 255 };
    constexpr Color BUSH_COLOR = { 76, 126, 84, 255 };

    // In pixels on the screen, smaller houses are left out, and so are the things around houses and in gardens while a house as wide as
    // DETAIL_WIDTH is smaller than DETAIL_PIXELS
    constexpr float MIN_HOUSE_PIXELS = 3.0f;
    constexpr float DETAIL_WIDTH = 10.0f;
    constexpr float DETAIL_PIXELS = 14.0f;
    constexpr float EDGE_PIXELS = 1.0f;

    const float edge = EDGE_PIXELS * worldPerPixel;
    const bool details = DETAIL_WIDTH >= DETAIL_PIXELS * worldPerPixel;
    const bool shown = house.built && 2.0f * house.halfSize.x >= MIN_HOUSE_PIXELS * worldPerPixel;
    const Vector2 across = { -house.axis.y, house.axis.x };
    const float driveway = Vector2Distance(house.drivewayStart, house.drivewayEnd);

    if (shown && driveway > 0.0f) drawBox(Vector2Lerp(house.drivewayStart, house.drivewayEnd, 0.5f), house.axis, { DRIVEWAY_HALF_WIDTH, 0.5f * driveway }, DRIVEWAY_COLOR);
    for (int i = 0; i < house.propCount && shown && details; ++i)
    {
        const Prop &prop = house.props[i];
        switch (prop.kind)
        {
            case PropKind_Pool:
            {
                drawBox(prop.position, house.axis, prop.halfSize + Vector2{ 0.3f, 0.3f }, POOL_COLOR);
                drawBox(prop.position, house.axis, prop.halfSize, WATER_COLOR);
                break;
            }
            case PropKind_Shed:
            {
                drawBox(prop.position, house.axis, prop.halfSize + Vector2{ edge, edge }, EDGE_COLOR);
                drawBox(prop.position, house.axis, prop.halfSize, SHED_COLOR);
                break;
            }
            case PropKind_Car:
            {
                // The glass is a band across the car under its roof
                const Color body = CAR_COLORS[prop.color % (int)std::size(CAR_COLORS)];
                drawBox(prop.position, house.axis, prop.halfSize, body);
                drawBox(prop.position, house.axis, prop.halfSize * Vector2{ 0.85f, 0.5f }, GLASS_COLOR);
                drawBox(prop.position, house.axis, prop.halfSize * Vector2{ 0.75f, 0.3f }, body);
                break;
            }
            default: break;
        }
    }

    // Far away a house is one box. Close up the two sides of its roof meet at the ridge, the one away from the light is darker
    const Color roof = ROOF_COLORS[house.roof % (int)std::size(ROOF_COLORS)];
    if (shown && !details) drawBox(house.center, house.axis, house.halfSize, roof);
    if (!details) return;
    if (shown)
    {
        const bool wide = house.halfSize.x >= house.halfSize.y;
        const Vector2 half = wide ? Vector2{ house.halfSize.x, 0.5f * house.halfSize.y } : Vector2{ 0.5f * house.halfSize.x, house.halfSize.y };
        const Vector2 shift = wide ? across * (0.5f * house.halfSize.y) : house.axis * (0.5f * house.halfSize.x);
        const Vector2 ridge = wide ? house.axis * house.halfSize.x : across * house.halfSize.y;
        drawBox(house.center, house.axis, house.halfSize + Vector2{ edge, edge }, EDGE_COLOR);
        drawBox(house.center - shift, house.axis, half, roof);
        drawBox(house.center + shift, house.axis, half, ColorBrightness(roof, -0.25f));
        DrawLineEx(house.center - ridge, house.center + ridge, edge, ColorBrightness(roof, 0.25f));
    }

    for (int i = 0; i < house.propCount; ++i)
    {
        const Prop &prop = house.props[i];
        if (prop.kind == PropKind_Bush) DrawPoly(prop.position, 8, prop.halfSize.x, 0.0f, BUSH_COLOR);
        if (prop.kind != PropKind_Tree) continue;

        // The light falls from the top left onto the tree tops
        DrawPoly(prop.position, 12, prop.halfSize.x, 0.0f, TREE_COLOR);
        DrawPoly(prop.position - Vector2{ 0.25f, 0.25f } * prop.halfSize.x, 10, 0.55f * prop.halfSize.x, 0.0f, LEAVES_COLOR);
    }
}

// ---- Road drawing

// The point at a distance along a line with the arc lengths of its samples, the piece it is on only moves forward
static Vector2 pointAlong(const Polyline &line, const std::array<float, CURVE_POINTS> &arc, float distance, int &piece)
{
    while (piece + 2 < line.count && arc[piece + 1] < distance) ++piece;
    const float length = arc[piece + 1] - arc[piece];
    return Vector2Lerp(line.points[piece], line.points[piece + 1], length > 0.0f ? Clamp((distance - arc[piece]) / length, 0.0f, 1.0f) : 0.0f);
}

// A dashed line along a road that starts startSkip after its first end and stops endSkip before its other end
static void drawDashes(const Polyline &line, float startSkip, float endSkip, float dash, float gap, float thickness, Color color)
{
    std::array<float, CURVE_POINTS> arc = {};
    for (int i = 1; i < line.count; ++i) arc[i] = arc[i - 1] + Vector2Distance(line.points[i - 1], line.points[i]);

    int piece = 0;
    const float end = arc[line.count - 1] - endSkip;
    for (float start = startSkip; start < end; start += dash + gap)
    {
        const Vector2 a = pointAlong(line, arc, start, piece);
        DrawLineEx(a, pointAlong(line, arc, fminf(start + dash, end), piece), thickness, color);
    }
}

// ---- Types

float Random::Next()
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return (float)(state & 0xFFFFFF) / 16777216.0f;
}

bool Cell::Add(Vector2 point, bool isStreet)
{
    if (count > 0 && Vector2Distance(points[count - 1], point) < 0.001f)
    {
        street[count - 1] = isStreet;
        return true;
    }
    if (count == MAX_CELL_POINTS) return false;

    points[count] = point;
    street[count++] = isStreet;
    return true;
}

// ---- City: queries

RoadAnchor City::Snap(Vector2 point, float radius) const
{
    int node = 0;
    if (FindNode(point, radius, node)) return { AnchorKind::Node, node, 0, 0.0f, nodes[node] };

    RoadAnchor anchor = { AnchorKind::Free, 0, 0, 0.0f, point };
    float bestSquared = radius * radius;
    for (int r = 0; r < roadCount; ++r)
    {
        float t = 0.0f;
        Vector2 closest = point;
        if (!contains(inflate(roads[r].box, radius), point) || closestOnLine(roads[r].line, point, t, closest) > bestSquared) continue;

        bestSquared = Vector2DistanceSqr(closest, point);
        anchor = { AnchorKind::Road, 0, r, t, closest };
    }
    return anchor;
}

bool City::FindNode(Vector2 point, float radius, int &node) const
{
    bool found = false;
    float bestSquared = radius * radius;
    for (int n = 0; n < nodeCount; ++n)
    {
        if (Vector2DistanceSqr(nodes[n], point) > bestSquared) continue;

        bestSquared = Vector2DistanceSqr(nodes[n], point);
        node = n;
        found = true;
    }
    return found;
}

bool City::FindRoad(Vector2 point, float radius, int &road) const
{
    bool found = false;
    float bestSquared = FLT_MAX;
    for (int r = 0; r < roadCount; ++r)
    {
        const float reach = fmaxf(radius, roads[r].attributes.width * 0.5f);
        float t = 0.0f;
        Vector2 closest = point;
        const float distanceSquared = contains(inflate(roads[r].box, reach), point) ? closestOnLine(roads[r].line, point, t, closest) : FLT_MAX;
        if (distanceSquared > reach * reach || distanceSquared >= bestSquared) continue;

        bestSquared = distanceSquared;
        road = r;
        found = true;
    }
    return found;
}

Box City::Bounds() const
{
    return nodeCount == 0 ? Box{ { 0.0f, 0.0f }, { 160.0f, 100.0f } } : boxOf({ nodes.data(), (size_t)nodeCount });
}

bool City::CanAddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road, bool &alongRoad) const
{
    alongRoad = false;
    const Polyline line = roadLine(a.position, road, b.position);
    if (Vector2Distance(a.position, b.position) < MIN_ROAD_LENGTH || curveTouchesItself(line, road.width)) return false;

    const Box box = boxOf(samples(line));
    for (int r = 0; r < roadCount && !alongRoad; ++r)
    {
        const float gap = road.width + roads[r].attributes.width;
        alongRoad = overlap(inflate(box, gap), roads[r].box) && curvesRunAlong(line, roads[r].line, gap, 0.5f * gap);
    }
    return !alongRoad;
}

bool City::CanMoveNode(int node, Vector2 position) const
{
    // Room kept between the intersection and other roads and intersections, besides their half widths
    constexpr float CLEARANCE = 0.5f;

    std::array<Polyline, MAX_MOVED_ROADS> lines;
    std::array<int, MAX_MOVED_ROADS> incident = {};
    float width = 0.0f;
    const int count = std::isfinite(position.x) && std::isfinite(position.y) ? PreviewMovedNode(node, position, lines, incident, width) : 0;
    if (count == 0) return false;

    // Its roads stay valid roads that neither cross nor run along each other
    Box movedBox;
    for (int i = 0; i < count; ++i)
    {
        const RoadAttributes &road = roads[incident[i]].attributes;
        if (Vector2Distance(lines[i].points[0], lines[i].points[lines[i].count - 1]) < MIN_ROAD_LENGTH || curveTouchesItself(lines[i], road.width)) return false;

        for (int j = i + 1; j < count; ++j)
        {
            const float gap = road.width + roads[incident[j]].attributes.width;
            if (linesCross(samples(lines[i]), samples(lines[j])) || curvesRunAlong(lines[i], lines[j], gap, 0.5f * gap)) return false;
        }

        const Box box = boxOf(samples(lines[i]));
        movedBox = { Vector2Min(movedBox.min, box.min), Vector2Max(movedBox.max, box.max) };
    }

    for (int n = 0; n < nodeCount; ++n)
    {
        if (n != node && Vector2Distance(nodes[n], position) < 0.5f * (nodeWidth[node] + nodeWidth[n]) + CLEARANCE) return false;
    }

    // The intersection keeps clear of the other roads, and its roads do not cross them or run along them unless they meet at an end
    for (int s = 0; s < roadCount; ++s)
    {
        const Road &other = roads[s];
        if (other.from == node || other.to == node) continue;

        const float reach = 0.5f * (nodeWidth[node] + other.attributes.width) + CLEARANCE;
        for (int i = 0; i + 1 < other.line.count && contains(inflate(other.box, reach), position); ++i)
        {
            if (pointSegmentDistance(position, other.line.points[i], other.line.points[i + 1]) < reach) return false;
        }
        if (!overlap(inflate(movedBox, nodeWidth[node] + other.attributes.width), other.box)) continue;

        for (int i = 0; i < count; ++i)
        {
            const Road &moved = roads[incident[i]];
            const bool adjacent = other.from == moved.from || other.from == moved.to || other.to == moved.from || other.to == moved.to;
            const float gap = moved.attributes.width + other.attributes.width;
            if (linesCross(samples(lines[i]), samples(other.line)) || (!adjacent && curvesRunAlong(lines[i], other.line, gap, 0.5f * gap))) return false;
        }
    }
    return true;
}

bool City::ParcelAt(Vector2 point, int &parcel) const
{
    // From the smallest parcel, so a parcel inside another one wins
    for (int i = parcelCount - 1; i >= 0; --i)
    {
        if (!contains(parcels[i].box, point) || !insidePolygon(ParcelOutline(i), point)) continue;

        parcel = i;
        return true;
    }
    return false;
}

bool City::FindBlock(Vector2 point, bool wholeBlock, Selection &selection) const
{
    selection.parcelCount = 0;
    selection.roadCount = 0;
    int index = 0;
    if (!ParcelAt(point, index)) return false;

    selection.parcels[selection.parcelCount++] = index;
    const unsigned block = parcels[index].attributes.block;
    if (!wholeBlock || !autoGenerate || block == 0) return true;

    // The block spreads from the parcel over the streets of its generation
    std::array<bool, MAX_PARCELS> inBlock = {};
    inBlock[index] = true;
    for (bool spread = true; spread;)
    {
        spread = false;
        for (int r = 0; r < roadCount; ++r)
        {
            const int left = halfedges[2 * r].parcel;
            const int right = halfedges[2 * r + 1].parcel;
            if (!JoinsBlock(r, block) || inBlock[left] == inBlock[right]) continue;

            inBlock[left] = inBlock[right] = true;
            spread = true;
        }
    }
    for (int i = 0; i < parcelCount; ++i)
    {
        if (i != index && inBlock[i]) selection.parcels[selection.parcelCount++] = i;
    }
    for (int r = 0; r < roadCount; ++r)
    {
        if (JoinsBlock(r, block) && inBlock[halfedges[2 * r].parcel]) selection.roads[selection.roadCount++] = r;
    }
    return true;
}

bool City::LotAt(Vector2 point, int &lot) const
{
    for (int l = 0; l < lotCount; ++l)
    {
        if (!contains(lots[l].box, point) || !insidePolygon(LotOutline(l), point)) continue;

        lot = l;
        return true;
    }
    return false;
}

std::span<const Vector2> City::ParcelOutline(int parcel) const
{
    return { parcelPoints.data() + parcels[parcel].firstPoint, (size_t)parcels[parcel].pointCount };
}

std::span<const Vector2> City::LotOutline(int lot) const
{
    return { lotPoints.data() + lots[lot].firstPoint, (size_t)lots[lot].pointCount };
}

// ---- City: edits

bool City::Overflow(const char *message)
{
    if (error == nullptr) error = message;
    return false;
}

bool City::AddRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road)
{
    bool alongRoad = false;
    if (!CanAddRoad(a, b, road, alongRoad) || !InsertRoad(a, b, road)) return false;

    Rebuild(true);
    return true;
}

bool City::MoveNode(int node, Vector2 position)
{
    return MoveNodes({ &node, 1 }, { &position, 1 });
}

bool City::MoveNodes(std::span<const int> list, std::span<const Vector2> positions)
{
    bool moved = false;
    for (size_t i = 0; i < list.size(); ++i)
    {
        const int node = list[i];
        if (!CanMoveNode(node, positions[i])) continue;

        nodes[node] = positions[i];
        for (int r = 0; r < roadCount; ++r)
        {
            if (roads[r].from != node && roads[r].to != node) continue;

            RefreshRoad(r);
            halfedges[2 * r].fresh = true;
            halfedges[2 * r + 1].fresh = true;
        }
        moved = true;
    }

    if (moved) Rebuild(true);
    return moved;
}

bool City::RemoveRoadAt(Vector2 point, float radius)
{
    int road = 0;
    if (!FindRoad(point, radius, road)) return false;

    // Without a generated road the parcel gets no streets, they would bring it back
    const bool generated = roads[road].attributes.generation != 0;
    RemoveRoads({ &road, 1 });
    Rebuild(!generated);
    return true;
}

bool City::ApplyParcelTypeAt(Vector2 point, SubdivisionStyle style, bool wholeBlock)
{
    Selection selection;
    if (!FindBlock(point, wholeBlock, selection)) return false;

    for (int i = 0; i < selection.parcelCount; ++i)
    {
        const int parcel = selection.parcels[i];
        ParcelAttributes attributes = parcels[parcel].attributes;
        attributes.style = style;
        SetAttributes(parcel, attributes);
        if (autoGenerate) MarkLoop(parcels[parcel].halfedge);
    }
    if (!autoGenerate) return true;

    // Without its streets the block is one parcel again, generated with the streets of its type
    RemoveRoads({ selection.roads.data(), (size_t)selection.roadCount });
    Rebuild(true);
    return true;
}

// ---- City: roads

int City::Origin(int halfedge) const
{
    return (halfedge & 1) ? roads[halfedge >> 1].to : roads[halfedge >> 1].from;
}

Vector2 City::HalfedgePoint(int halfedge, int i) const
{
    const Polyline &line = roads[halfedge >> 1].line;
    return line.points[(halfedge & 1) ? line.count - 1 - i : i];
}

bool City::InsertRoad(const RoadAnchor &a, const RoadAnchor &b, const RoadAttributes &road)
{
    // Touching the end of a road counts, so a road can end exactly on another road
    constexpr float TOUCH = 0.0001f;
    constexpr int MAX_HITS = 256;

    const Polyline line = roadLine(a.position, road, b.position);
    const Box box = boxOf(samples(line));

    std::array<Hit, MAX_HITS> hits;
    int hitCount = 0;
    for (int r = 0; r < roadCount; ++r)
    {
        if (overlap(box, roads[r].box)) hitCount = lineHits(line, roads[r].line, r, TOUCH, hits, hitCount);
    }

    // Every hit splits a road at a new node, the ends can split two more, and the new road has a piece per stop. With hits left out it
    // would cross roads without splitting them
    if (hitCount == MAX_HITS) return Overflow("MAX_HITS reached, the road crosses too many roads");
    if (nodeCount + hitCount + 2 > MAX_NODES) return Overflow("MAX_NODES reached, roads were left out");
    if (roadCount + 2 * hitCount + 3 > MAX_ROADS) return Overflow("MAX_ROADS reached, roads were left out");

    // Every road is split from its far end on, so the first half keeps its index. A crossing near an end of a road is that end
    std::sort(hits.begin(), hits.begin() + hitCount, [](const Hit &first, const Hit &second) { return first.road != second.road ? first.road < second.road : first.t > second.t; });
    std::array<Stop, MAX_HITS + 2> stops;
    int stopCount = 0;
    float scale = 1.0f;
    for (int k = 0; k < hitCount; ++k)
    {
        if (k == 0 || hits[k].road != hits[k - 1].road) scale = 1.0f;

        const Road &crossed = roads[hits[k].road];
        const Vector2 point = bezierPoint(roadBezier(nodes[crossed.from], crossed.attributes, nodes[crossed.to]), hits[k].t / scale);
        int node = Vector2Distance(point, nodes[crossed.from]) < Vector2Distance(point, nodes[crossed.to]) ? crossed.from : crossed.to;
        if (Vector2Distance(point, nodes[node]) >= MERGE_DISTANCE)
        {
            node = SplitRoad(hits[k].road, hits[k].t / scale);
            scale = hits[k].t;
        }
        stops[stopCount++] = { hits[k].along, node };
    }

    // The ends, then the stops in order along the road, a node that comes twice in a row is kept once
    stops[stopCount++] = { 0.0f, NodeAt(a.position) };
    stops[stopCount++] = { 1.0f, NodeAt(b.position) };
    std::sort(stops.begin(), stops.begin() + stopCount, [](const Stop &first, const Stop &second) { return first.along < second.along; });

    int kept = 1;
    for (int k = 1; k < stopCount; ++k)
    {
        if (stops[k].node != stops[kept - 1].node) stops[kept++] = stops[k];
    }

    stopCount = kept;
    stops[0].along = 0.0f;
    stops[stopCount - 1].along = 1.0f;

    // One road between every two neighboring stops cut from the Bezier of the new road, a piece inside a parcel takes its type on both sides
    Bezier rest = roadBezier(a.position, road, b.position);
    float consumed = 0.0f;
    for (int j = 0; j + 1 < stopCount; ++j)
    {
        Bezier piece = rest;
        if (j + 2 < stopCount) splitBezier(rest, (stops[j + 1].along - consumed) / fmaxf(1.0f - consumed, 0.000001f), piece, rest);
        consumed = stops[j + 1].along;

        // A piece over a road between the same intersections is that road
        const int from = stops[j].node;
        const int to = stops[j + 1].node;

        bool same = false;
        for (int r = 0; r < roadCount && !same; ++r)
        {
            same = ((roads[r].from == from && roads[r].to == to) || (roads[r].from == to && roads[r].to == from)) && Vector2Distance(lineMiddle(roads[r].line), bezierPoint(piece, 0.5f)) < MERGE_DISTANCE;
        }
        if (same) continue;

        RoadAttributes attributes = road;
        attributes.controls = { piece.points[1], piece.points[2] };
        AppendRoad(from, to, attributes);

        int around = 0;
        if (!ParcelAt(lineMiddle(roads[roadCount - 1].line), around)) continue;

        for (int side = 0; side < 2; ++side)
        {
            halfedges[2 * (roadCount - 1) + side].attributes = parcels[around].attributes;
            halfedges[2 * (roadCount - 1) + side].hasAttributes = true;
        }
    }

    RefreshNodes();
    return true;
}

void City::AppendRoad(int from, int to, const RoadAttributes &attributes)
{
    roads[roadCount] = { from, to, attributes, Polyline(), Box() };
    halfedges[2 * roadCount] = Halfedge();
    halfedges[2 * roadCount + 1] = Halfedge();
    RefreshRoad(roadCount++);
}

void City::RefreshRoad(int road)
{
    Road &r = roads[road];
    r.line = roadLine(nodes[r.from], r.attributes, nodes[r.to]);
    r.box = inflate(boxOf(samples(r.line)), r.attributes.width);
}

int City::NodeAt(Vector2 point)
{
    const RoadAnchor anchor = Snap(point, MERGE_DISTANCE);
    if (anchor.kind == AnchorKind::Node) return anchor.node;
    if (anchor.kind == AnchorKind::Road) return SplitRoad(anchor.road, anchor.t);

    nodes[nodeCount] = point;
    return nodeCount++;
}

int City::SplitRoad(int road, float t)
{
    // An intersection closer than this to where the road is cut is the place it is cut at
    constexpr float SAME_PLACE = 0.05f;

    Bezier first;
    Bezier second;
    splitBezier(roadBezier(nodes[roads[road].from], roads[road].attributes, nodes[roads[road].to]), t, first, second);

    // A second intersection at the same place would make roads that run twice
    int node = 0;
    if (!FindNode(first.points[3], SAME_PLACE, node) || node == roads[road].from || node == roads[road].to)
    {
        node = nodeCount;
        nodes[nodeCount++] = first.points[3];
    }

    // Both halves keep the sides of the road
    RoadAttributes rest = roads[road].attributes;
    rest.controls = { second.points[1], second.points[2] };
    const int to = roads[road].to;
    roads[road].to = node;
    roads[road].attributes.controls = { first.points[1], first.points[2] };
    RefreshRoad(road);
    AppendRoad(node, to, rest);
    halfedges[2 * roadCount - 2] = halfedges[2 * road];
    halfedges[2 * roadCount - 1] = halfedges[2 * road + 1];
    return node;
}

void City::RemoveRoads(std::span<const int> list)
{
    // From the last one on, every road swaps places with the last road, which is not one still to go
    std::array<int, MAX_ROADS> order;
    std::copy(list.begin(), list.end(), order.begin());
    std::sort(order.begin(), order.begin() + (long)list.size(), [](int first, int second) { return first > second; });
    const int count = (int)(std::unique(order.begin(), order.begin() + (long)list.size()) - order.begin());

    // The faces on both sides become one, all of their sides change
    Link();
    for (int k = 0; k < count; ++k)
    {
        MarkLoop(2 * order[k]);
        MarkLoop(2 * order[k] + 1);
    }

    std::array<int, 2 * MAX_ROADS> ends;
    for (int k = 0; k < count; ++k)
    {
        const int r = order[k];
        ends[2 * k] = roads[r].from;
        ends[2 * k + 1] = roads[r].to;

        --roadCount;

        roads[r] = roads[roadCount];
        halfedges[2 * r] = halfedges[2 * roadCount];
        halfedges[2 * r + 1] = halfedges[2 * roadCount + 1];
    }
    for (int k = 0; k < 2 * count; ++k) MergeStraightRoads(ends[k]);
    RefreshNodes();
}

void City::MergeStraightRoads(int node)
{
    constexpr float COLLINEAR = 0.0001f;

    std::array<int, 3> incident = {};
    int count = 0;
    for (int r = 0; r < roadCount && count < 3; ++r)
    {
        if (roads[r].from == node || roads[r].to == node) incident[count++] = r;
    }
    if (count != 2) return;

    Road &first = roads[incident[0]];
    const Road &second = roads[incident[1]];
    if (first.attributes.curved || second.attributes.curved || first.attributes.width != second.attributes.width || first.attributes.generation != second.attributes.generation) return;

    const int endA = first.from == node ? first.to : first.from;
    const int endB = second.from == node ? second.to : second.from;
    const Vector2 toA = nodes[endA] - nodes[node];
    const Vector2 toB = nodes[endB] - nodes[node];
    if (endA == endB || Vector2DotProduct(toA, toB) >= 0.0f || fabsf(Vector2CrossProduct(toA, toB)) > COLLINEAR * Vector2Length(toA) * Vector2Length(toB)) return;

    // The first road runs from endA to endB now, a road that left the node turns round and its sides swap
    if (first.from == node) std::swap(halfedges[2 * incident[0]], halfedges[2 * incident[0] + 1]);

    first.from = endA;
    first.to = endB;
    RefreshRoad(incident[0]);

    --roadCount;
    
    roads[incident[1]] = roads[roadCount];
    halfedges[2 * incident[1]] = halfedges[2 * roadCount];
    halfedges[2 * incident[1] + 1] = halfedges[2 * roadCount + 1];
}

void City::RefreshNodes()
{
    std::fill(nodeWidth.begin(), nodeWidth.begin() + nodeCount, 0.0f);
    std::fill(nodeDegree.begin(), nodeDegree.begin() + nodeCount, 0);

    for (int r = 0; r < roadCount; ++r)
    {
        for (int node : { roads[r].from, roads[r].to })
        {
            nodeWidth[node] = fmaxf(nodeWidth[node], roads[r].attributes.width);
            ++nodeDegree[node];
        }
    }

    std::array<int, MAX_NODES> renumbered;
    int kept = 0;
    for (int n = 0; n < nodeCount; ++n)
    {
        if (nodeDegree[n] == 0) continue;

        renumbered[n] = kept;
        nodes[kept] = nodes[n];
        nodeWidth[kept] = nodeWidth[n];
        nodeDegree[kept++] = nodeDegree[n];
    }

    nodeCount = kept;
    for (int r = 0; r < roadCount; ++r)
    {
        roads[r].from = renumbered[roads[r].from];
        roads[r].to = renumbered[roads[r].to];
    }
}

void City::RemoveHanging(unsigned generation)
{
    // Every pass removes roads, a pass that removes none ends it
    while (true)
    {
        // Nodes that roads of other generations reach, spread over the roads of this one
        std::array<bool, MAX_NODES> reached = {};
        for (int r = 0; r < roadCount; ++r)
        {
            if (roads[r].attributes.generation != generation) reached[roads[r].from] = reached[roads[r].to] = true;
        }

        for (bool spread = true; spread;)
        {
            spread = false;
            for (int r = 0; r < roadCount; ++r)
            {
                if (roads[r].attributes.generation != generation || reached[roads[r].from] == reached[roads[r].to]) continue;

                reached[roads[r].from] = reached[roads[r].to] = true;
                spread = true;
            }
        }

        std::array<int, MAX_ROADS> hanging;
        int count = 0;
        for (int r = 0; r < roadCount; ++r)
        {
            const Road &road = roads[r];
            if (road.attributes.generation == generation && (nodeDegree[road.from] == 1 || nodeDegree[road.to] == 1 || !reached[road.from])) hanging[count++] = r;
        }
        if (count == 0) return;

        const int before = roadCount;
        RemoveRoads({ hanging.data(), (size_t)count });
        if (roadCount == before) return;
    }
}

int City::PreviewMovedNode(int node, Vector2 position, std::array<Polyline, MAX_MOVED_ROADS> &lines, std::array<int, MAX_MOVED_ROADS> &incident, float &width) const
{
    width = 0.0f;
    if (node < 0 || node >= nodeCount || nodeDegree[node] > MAX_MOVED_ROADS) return 0;

    int count = 0;
    for (int r = 0; r < roadCount; ++r)
    {
        const Road &road = roads[r];
        if (road.from != node && road.to != node) continue;

        incident[count] = r;
        lines[count++] = roadLine(road.from == node ? position : nodes[road.from], road.attributes, road.to == node ? position : nodes[road.to]);
        width = fmaxf(width, road.attributes.width);
    }

    return count;
}

// ---- City: parcels

void City::Link()
{
    // Halfedges by the node they leave and by their angle. The face on the left of a halfedge goes on with the next road clockwise
    // around the node it reaches, the one before its way back in that order
    std::array<int, MAX_HALFEDGES> order = { 0 };
    std::array<float, MAX_HALFEDGES> angles = { 0.0f };

    const int count = 2 * roadCount;
    for (int h = 0; h < count; ++h)
    {
        const Vector2 direction = HalfedgePoint(h, 1) - HalfedgePoint(h, 0);
        angles[h] = atan2f(direction.y, direction.x);
        order[h] = h;
    }

    std::sort(order.begin(), order.begin() + count, [&](int first, int second) {
        if (Origin(first) != Origin(second)) return Origin(first) < Origin(second);
        return angles[first] != angles[second] ? angles[first] < angles[second] : first < second;
    });

    for (int start = 0, end = 0; start < count; start = end)
    {
        for (end = start; end < count && Origin(order[end]) == Origin(order[start]); ++end) {}
        for (int i = start; i < end; ++i) halfedges[order[i] ^ 1].next = order[i > start ? i - 1 : end - 1];
    }
}

void City::Trace()
{
    constexpr float MIN_AREA = 0.01f;
    constexpr int UNTRACED = -2;

    Link();
    ++meshVersion;
    for (int h = 0; h < 2 * roadCount; ++h) halfedges[h].parcel = UNTRACED;

    // Every loop of positive area is a parcel, loops around the outside and around holes run clockwise
    parcelCount = 0;
    int pointCount = 0;
    int indexCount = 0;
    for (int start = 0; start < 2 * roadCount; ++start)
    {
        if (halfedges[start].parcel != UNTRACED) continue;

        const std::span<Vector2> room = { parcelPoints.data() + pointCount, (size_t)std::min(MAX_PARCEL_POINTS - pointCount, MAX_PARCEL_CORNERS) };
        const bool face = LoopArea(start) > MIN_AREA;
        const int points = face && parcelCount < MAX_PARCELS ? LoopOutline(start, room) : 0;

        int h = start;
        if (points < 3)
        {
            // A face that does not fit is left out
            if (face && parcelCount == MAX_PARCELS) Overflow("MAX_PARCELS reached, parcels were left out");
            if (face) Overflow((int)room.size() < MAX_PARCEL_CORNERS ? "MAX_PARCEL_POINTS reached, parcels were left out" : "MAX_PARCEL_CORNERS reached, a parcel was left out");
            do
            {
                halfedges[h].parcel = -1;
                halfedges[h].hasAttributes = false;
                h = halfedges[h].next;
            } while (h != start);
            continue;
        }

        Parcel &parcel = parcels[parcelCount];
        const std::span<const Vector2> outline = room.first((size_t)points);
        parcel = Parcel();
        parcel.halfedge = start;
        parcel.firstPoint = pointCount;
        parcel.pointCount = points;
        parcel.firstIndex = indexCount;
        parcel.area = signedArea(outline);
        parcel.box = boxOf(outline);
        parcel.probe = interiorPoint(outline);
        pointCount += points;

        // Triangles counter-clockwise like the outline
        parcel.indexCount = triangulate(outline, { parcelIndices.data() + indexCount, (size_t)(MAX_PARCEL_INDICES - indexCount) });
        indexCount += parcel.indexCount;

        // The type of a side that did not change wins over the type of a new one
        int from = -1;
        do
        {
            parcel.changed = parcel.changed || halfedges[h].fresh;
            if (halfedges[h].hasAttributes && (from < 0 || (halfedges[from].fresh && !halfedges[h].fresh))) from = h;
            halfedges[h].parcel = parcelCount;
            h = halfedges[h].next;
        } while (h != start);

        if (from >= 0) parcel.attributes = halfedges[from].attributes;
        else parcel.attributes.style = newParcelStyle;
        SetAttributes(parcelCount++, parcel.attributes);
    }

    // From the largest to the smallest, then every halfedge learns the new index of its parcel
    std::sort(parcels.begin(), parcels.begin() + parcelCount, [](const Parcel &first, const Parcel &second) { return first.area > second.area; });
    for (int i = 0; i < parcelCount; ++i)
    {
        int h = parcels[i].halfedge;
        do
        {
            halfedges[h].parcel = i;
            h = halfedges[h].next;
        } while (h != parcels[i].halfedge);
    }

    // The probe of a parcel around another one can be in the inner one, it moves next to one of its sides then
    for (int i = 0; i < parcelCount; ++i)
    {
        const std::span<const Vector2> outline = ParcelOutline(i);
        int at = i;
        for (size_t k = 0; k < outline.size() && !(ParcelAt(parcels[i].probe, at) && at == i); ++k)
        {
            const Vector2 next = outline[(k + 1) % outline.size()];
            parcels[i].probe = Vector2Lerp(outline[k], next, 0.5f) + leftNormal(outline[k], next) * 0.01f;
        }
    }
}

float City::LoopArea(int start) const
{
    // Summed around the first node, so that far out the sum keeps its precision
    const Vector2 origin = nodes[Origin(start)];
    float area = 0.0f;
    int h = start;
    do
    {
        for (int i = 0; i + 1 < roads[h >> 1].line.count; ++i) area += 0.5f * Vector2CrossProduct(HalfedgePoint(h, i) - origin, HalfedgePoint(h, i + 1) - origin);
        h = halfedges[h].next;
    } while (h != start);
    return area;
}

int City::LoopOutline(int start, std::span<Vector2> out) const
{
    // The roads inset a little to the left, a dead end becomes a narrow slit and a road that leads to a hole a cut to it
    constexpr float INSET = 0.02f;

    int previous = start;
    for (int h = halfedges[start].next; h != start; h = halfedges[h].next) previous = h;

    int count = 0;
    int h = start;
    do
    {
        const int sampleCount = roads[h >> 1].line.count;
        for (int i = 0; i + 1 < sampleCount; ++i)
        {
            if (count == (int)out.size()) return 0;

            const Vector2 before = i > 0 ? HalfedgePoint(h, i - 1) : HalfedgePoint(previous, roads[previous >> 1].line.count - 2);
            out[count++] = offsetCorner(before, HalfedgePoint(h, i), HalfedgePoint(h, i + 1), INSET);
        }
        previous = h;
        h = halfedges[h].next;
    } while (h != start);

    return count;
}

void City::MarkLoop(int start)
{
    int h = start;
    do
    {
        halfedges[h].fresh = true;
        h = halfedges[h].next;
    } while (h != start);
}

void City::SetAttributes(int parcel, const ParcelAttributes &attributes)
{
    parcels[parcel].attributes = attributes;
    int h = parcels[parcel].halfedge;
    do
    {
        halfedges[h].attributes = attributes;
        halfedges[h].hasAttributes = true;
        h = halfedges[h].next;
    } while (h != parcels[parcel].halfedge);
}

void City::ClearFresh()
{
    for (int h = 0; h < 2 * roadCount; ++h) halfedges[h].fresh = false;
}

bool City::JoinsBlock(int road, unsigned block) const
{
    const int left = halfedges[2 * road].parcel;
    const int right = halfedges[2 * road + 1].parcel;
    return roads[road].attributes.generation == block && left >= 0 && right >= 0 && parcels[left].attributes.block == block && parcels[right].attributes.block == block;
}

void City::Rebuild(bool withStreets)
{
    Trace();

    // Lots stay in the parcels that kept their shape
    int kept = 0;
    int keptPoints = 0;
    for (int l = 0; l < lotCount; ++l)
    {
        int at = 0;
        if (!ParcelAt(lots[l].probe, at) || parcels[at].changed) continue;

        const int first = lots[l].firstPoint;
        const int count = lots[l].pointCount;
        std::copy(lotPoints.begin() + first, lotPoints.begin() + first + count, lotPoints.begin() + keptPoints);
        std::copy(lotStreets.begin() + first, lotStreets.begin() + first + count, lotStreets.begin() + keptPoints);

        lots[kept] = lots[l];
        lots[kept++].firstPoint = keptPoints;
        keptPoints += count;
    }

    lotCount = kept;
    lotPointCount = keptPoints;

    // Generating traces the parcels again, so the changed ones are found again by their probes
    std::array<Vector2, MAX_PARCELS> probes;
    int probeCount = 0;
    for (int i = 0; i < parcelCount; ++i)
    {
        if (parcels[i].changed) probes[probeCount++] = parcels[i].probe;
    }

    ClearFresh();
    for (int i = 0; i < probeCount && autoGenerate; ++i) GenerateAt(probes[i], withStreets);
}

void City::GenerateAt(Vector2 point, bool withStreets)
{
    int parcel = 0;
    if (!ParcelAt(point, parcel)) return;

    // Generated roads that lie inside the parcel, the dead ends to lots from before, go first
    std::array<int, MAX_ROADS> inside;
    int insideCount = 0;
    for (int r = 0; r < roadCount; ++r)
    {
        if (roads[r].attributes.generation != 0 && halfedges[2 * r].parcel == parcel && halfedges[2 * r + 1].parcel == parcel) inside[insideCount++] = r;
    }

    if (insideCount > 0)
    {
        RemoveRoads({ inside.data(), (size_t)insideCount });
        Trace();
        if (!ParcelAt(point, parcel)) return;
    }

    // A parcel that gets streets is the middle of its pattern, one that only gets lots stays in the pattern it was generated in
    const unsigned generation = nextGeneration++;
    ParcelAttributes attributes = parcels[parcel].attributes;
    if (withStreets || attributes.block == 0) attributes.center = parcels[parcel].probe;

    attributes.block = generation;
    attributes.settings = parcelSettings[(int)attributes.style];
    SetAttributes(parcel, attributes);

    // Streets of the type cut a large parcel into blocks that take its type, too narrow and too long blocks are fixed, then every block
    // gets lots
    if (withStreets) AddStreets(parcel, generation);
    RemoveHanging(generation);
    Trace();
    if (withStreets && attributes.settings.maxBlockSize >= MIN_BLOCK_SIZE) FitBlocks(generation);
    const int roadsBefore = roadCount;
    const int lotsBefore = lotCount;
    for (int i = 0; i < parcelCount; ++i)
    {
        if (parcels[i].attributes.block == generation && parcels[i].area >= attributes.settings.minParcelArea) FillLots(i, generation * 7919u + (unsigned)i);
    }

    // The streets that the lots needed cut the blocks into parcels. The new lots get their houses once all roads are there
    if (roadCount != roadsBefore) Trace();
    for (int l = lotsBefore; l < lotCount; ++l) BuildHouse(l, lots[l].house.seed);
    ClearFresh();
}

void City::AddStreets(int parcel, unsigned generation)
{
    constexpr int MAX_STREETS = 1024;

    std::array<Street, MAX_STREETS> streets;
    const ParcelAttributes &attributes = parcels[parcel].attributes;
    const int count = streetPattern(ParcelOutline(parcel), attributes, generation, streets);
    if (count == MAX_STREETS) Overflow("MAX_STREETS reached, streets were left out");
    for (int s = 0; s < count; ++s) LayStreetIn(parcel, streets[s].a, streets[s].b, streets[s].road, generation, true);
}

bool City::LayStreetIn(int parcel, Vector2 a, Vector2 b, const RoadAttributes &road, unsigned generation, bool lay)
{
    constexpr int MAX_CUTS = 128;

    // Ends closer than this to an intersection or a road end there, like in LayStreet
    constexpr float STREET_SNAP = 0.3f;

    // A street is cut where it crosses roads and its pieces in the parcel are laid. The parcels are traced again only after all streets,
    // so until then the parcel covers its whole place
    const Bezier curve = roadBezier(a, road, b);
    const Polyline line = roadLine(a, road, b);
    const Box box = boxOf(samples(line));
    std::array<Hit, MAX_CUTS> hits;
    int hitCount = 0;
    for (int r = 0; r < roadCount; ++r)
    {
        if (overlap(box, roads[r].box)) hitCount = lineHits(line, roads[r].line, r, 0.0f, hits, hitCount);
    }

    std::array<float, MAX_CUTS + 2> cuts = { 0.0f, 1.0f };
    for (int k = 0; k < hitCount; ++k) cuts[k + 2] = hits[k].along;
    std::sort(cuts.begin(), cuts.begin() + hitCount + 2);

    int pieces = 0;
    bool fits = true;
    for (int k = 0; k + 1 < hitCount + 2; ++k)
    {
        int at = 0;
        if (!ParcelAt(bezierPoint(curve, 0.5f * (cuts[k] + cuts[k + 1])), at) || at != parcel) continue;

        // A piece along a road would be a second road on top of it, stubs past the last crossing go with the dead ends
        const Bezier piece = subBezier(curve, cuts[k], cuts[k + 1]);
        RoadAttributes pieceRoad = road;
        pieceRoad.controls = { piece.points[1], piece.points[2] };
        pieceRoad.generation = generation;
        bool alongRoad = false;
        const bool fit = lay ? LayStreet(piece.points[0], piece.points[3], pieceRoad, generation) : CanAddRoad(Snap(piece.points[0], STREET_SNAP), Snap(piece.points[3], STREET_SNAP), pieceRoad, alongRoad);
        fits = fits && fit;
        ++pieces;
    }
    return pieces > 0 && fits;
}

void City::FitBlocks(unsigned generation)
{
    // Most rounds of merging and splitting, a round that changes nothing ends it before
    constexpr int MAX_ROUNDS = 8;

    for (int round = 0; round < MAX_ROUNDS; ++round)
    {
        const bool merged = MergeBlocks(generation, generation * 31u + (unsigned)round);
        const bool split = SplitLongBlocks(generation, generation * 37u + (unsigned)round);
        if (!merged && !split) return;
    }
}

bool City::MergeBlocks(unsigned generation, unsigned seed)
{
    // A block a hundredth narrower than minBlockSize counts as wide enough, like in blockSplits
    constexpr float TOLERANCE = 0.99f;
    constexpr int MAX_LINES = 16;

    Random random = { seed * 2654435761u | 1u };
    std::array<Line, MAX_LINES> lines;
    std::array<bool, MAX_PARCELS> merged = {};
    std::array<int, MAX_PARCELS> neighbors;
    std::array<int, MAX_ROADS> streets;
    int streetCount = 0;
    for (int p = 0; p < parcelCount; ++p)
    {
        const ParcelSettings &settings = parcels[p].attributes.settings;
        if (parcels[p].attributes.block != generation || merged[p]) continue;

        // Too narrow, or too long with no line across that leaves both parts wide enough, like a wedge along the parcel
        const std::span<const Vector2> outline = ParcelOutline(p);
        const OrientedBox box = hullBox(outline, {});
        const bool thin = box.width < TOLERANCE * settings.minBlockSize;
        const bool stuck = !thin && box.length > settings.maxBlockSize && blockSplits(outline, box, parcels[p].attributes, random, lines) == 0;
        if (!thin && !stuck) continue;

        // The blocks of the generation across its streets
        int neighborCount = 0;
        for (int r = 0; r < roadCount; ++r)
        {
            const int left = halfedges[2 * r].parcel;
            const int right = halfedges[2 * r + 1].parcel;
            const int other = left == p ? right : left;
            if (!JoinsBlock(r, generation) || (left != p && right != p) || other == p || merged[other]) continue;
            if (std::find(neighbors.begin(), neighbors.begin() + neighborCount, other) == neighbors.begin() + neighborCount) neighbors[neighborCount++] = other;
        }

        // It joins the neighbor that makes it widest, a thin block as wide as minBlockSize or twice as wide as it is, a stuck one wider by
        // minBlockSize. A merged block that is too long is cut across by SplitLongBlocks
        float bestWidth = thin ? fminf(settings.minBlockSize, 2.0f * box.width) : box.width + settings.minBlockSize;
        int best = -1;
        for (int k = 0; k < neighborCount; ++k)
        {
            const OrientedBox joint = hullBox(outline, ParcelOutline(neighbors[k]));
            if (joint.width < bestWidth) continue;

            bestWidth = joint.width;
            best = neighbors[k];
        }
        if (best < 0) continue;

        merged[p] = true;
        merged[best] = true;
        for (int r = 0; r < roadCount; ++r)
        {
            const int left = halfedges[2 * r].parcel;
            const int right = halfedges[2 * r + 1].parcel;
            if (JoinsBlock(r, generation) && ((left == p && right == best) || (left == best && right == p))) streets[streetCount++] = r;
        }
    }
    if (streetCount == 0) return false;

    RemoveRoads({ streets.data(), (size_t)streetCount });
    RemoveHanging(generation);
    Trace();
    return true;
}

bool City::SplitLongBlocks(unsigned generation, unsigned seed)
{
    // Lines a block is tried with at most, and how far an organic street bends across it by the width of the block
    constexpr int MAX_LINES = 16;
    constexpr float ORGANIC_BEND = 0.15f;

    Random random = { seed * 2246822519u | 1u };
    std::array<Line, MAX_LINES> lines;
    std::array<Street, 1> street;
    const int roadsBefore = roadCount;
    int blocks = 0;
    for (int p = 0; p < parcelCount; ++p)
    {
        const ParcelAttributes &attributes = parcels[p].attributes;
        if (attributes.block != generation) continue;

        ++blocks;
        const std::span<const Vector2> outline = ParcelOutline(p);
        const OrientedBox box = hullBox(outline, {});
        const int lineCount = box.length > attributes.settings.maxBlockSize ? blockSplits(outline, box, attributes, random, lines) : 0;

        // The best line whose street can be laid in full, one along a road cannot
        for (int k = 0; k < lineCount; ++k)
        {
            const Line &line = lines[k];
            if (addLine(outline, line.origin, line.direction, street, 0) == 0) continue;

            if (attributes.style == SubdivisionStyle_Organic)
            {
                const Vector2 bend = Vector2{ -line.direction.y, line.direction.x } * (ORGANIC_BEND * box.width * (2.0f * random.Next() - 1.0f));
                street[0].road = bentRoad(street[0].a, line.origin + bend, street[0].b);
            }
            if (!LayStreetIn(p, street[0].a, street[0].b, street[0].road, generation, false)) continue;

            LayStreetIn(p, street[0].a, street[0].b, street[0].road, generation, true);
            break;
        }
    }

    // Streets that reach no road go, so only a block that is cut in two counts
    if (roadCount == roadsBefore) return false;

    RemoveHanging(generation);
    Trace();
    int after = 0;
    for (int p = 0; p < parcelCount; ++p) after += parcels[p].attributes.block == generation;
    return after > blocks;
}

bool City::LayStreet(Vector2 a, Vector2 b, RoadAttributes road, unsigned generation)
{
    // Ends closer than this to an intersection or a road end there
    constexpr float STREET_SNAP = 0.3f;

    const RoadAnchor first = Snap(a, STREET_SNAP);
    const RoadAnchor second = Snap(b, STREET_SNAP);
    road.generation = generation;
    bool alongRoad = false;

    return CanAddRoad(first, second, road, alongRoad) && InsertRoad(first, second, road);
}

void City::FillLots(int parcel, unsigned seed)
{
    constexpr int MAX_WAITING = 32;

    // Every step splits a cell or ends one, so a parcel takes about twice as many steps as it gets lots. More steps than four times the
    // lots of the whole city only come from settings that let cells split on and on
    constexpr int MAX_STEPS = 4 * MAX_LOTS;

    const ParcelAttributes attributes = parcels[parcel].attributes;
    const ParcelSettings &settings = attributes.settings;
    Cell cell;
    if (!thinOutline(ParcelOutline(parcel), cell))
    {
        if (cell.count == MAX_CELL_POINTS) Overflow("MAX_CELL_POINTS reached, a parcel got no lots");
        return;
    }

    // Depth first, a cell that is small enough or cannot be split is a lot, the second part of a split waits. A street that a split
    // needs is laid at once, a cell where it cannot be laid is split without streets. Blocks of no size get no streets at all
    Random random = { seed * 2654435761u | 1u };
    const bool streets = settings.maxBlockSize >= MIN_BLOCK_SIZE;
    std::array<Cell, MAX_WAITING> waiting;
    std::array<Cell, 2> parts;
    int waitingCount = 0;
    for (int k = 0; k < MAX_STEPS; ++k)
    {
        const std::span<const Vector2> points = corners(cell);
        const OrientedBox box = orientedBox(points);
        const bool large = signedArea(points) > settings.maxParcelArea || box.width > settings.maxParcelWidth;
        bool street = false;
        Street chord;
        if (large && waitingCount == MAX_WAITING) Overflow("MAX_WAITING reached, lots were left too large");
        
        bool split = large && waitingCount < MAX_WAITING && splitCell(cell, attributes, streets, random, parts, street, chord);
        if (split && street && !LayStreet(chord.a, chord.b, chord.road, attributes.block)) split = splitCell(cell, attributes, false, random, parts, street, chord);
        if (split)
        {
            waiting[waitingCount++] = parts[1];
            cell = parts[0];
            continue;
        }

        // A cell narrower or smaller than the smallest lot is no lot, like a sliver between roads that the roads hide, nor is one over a hole
        int at = 0;
        const bool fits = box.width >= settings.minParcelWidth && signedArea(points) >= settings.minParcelArea;
        if (fits && ParcelAt(interiorPoint(points), at) && at == parcel && !RoadThrough(points))
        {
            AddLot(points, { cell.street.data(), (size_t)cell.count }, seed + (unsigned)lotCount);
        }
        if (waitingCount == 0) return;

        cell = waiting[--waitingCount];
    }
    Overflow("MAX_STEPS reached, lots were left out");
}

void City::AddLot(std::span<const Vector2> points, std::span<const bool> streets, unsigned seed)
{
    if (lotCount == MAX_LOTS || lotPointCount + (int)points.size() > MAX_LOT_POINTS)
    {
        Overflow(lotCount == MAX_LOTS ? "MAX_LOTS reached, lots were left out" : "MAX_LOT_POINTS reached, lots were left out");
        return;
    }

    lots[lotCount] = { lotPointCount, (int)points.size(), boxOf(points), interiorPoint(points), House() };
    lots[lotCount++].house.seed = seed;
    std::copy(points.begin(), points.end(), lotPoints.begin() + lotPointCount);
    std::copy(streets.begin(), streets.end(), lotStreets.begin() + lotPointCount);
    lotPointCount += (int)points.size();
}

void City::BuildHouse(int lot, unsigned seed)
{
    constexpr int ATTEMPTS = 10;
    constexpr int PLACES = 5;
    constexpr int MAX_NEAR_ROADS = 32;

    House &house = lots[lot].house;
    house = House();
    house.seed = seed;

    // The house faces the longest stretch of road along the lot, from a to b. Street sides that go on almost straight, like the samples
    // of a curve, are one stretch
    const std::span<const Vector2> outline = LotOutline(lot);
    const std::span<const bool> streets = { lotStreets.data() + lots[lot].firstPoint, outline.size() };
    const int n = (int)outline.size();

    std::array<Road, MAX_NEAR_ROADS> near;
    int nearCount = 0;
    for (int r = 0; r < roadCount && nearCount < MAX_NEAR_ROADS; ++r)
    {
        if (overlap(inflate(lots[lot].box, STREET_CLEARANCE), roads[r].box)) near[nearCount++] = roads[r];
    }

    const Ground ground = { outline, streets, { near.data(), (size_t)nearCount } };

    int first = 0;
    int end = 0;
    for (int i = 0; i < n; ++i)
    {
        if (!streets[i] || (streets[(i + n - 1) % n] && goesStraightOn(outline, (i + n - 1) % n))) continue;

        int last = i;
        while ((last + 1) % n != i && streets[(last + 1) % n] && goesStraightOn(outline, last)) last = (last + 1) % n;
        if (Vector2Distance(outline[i], outline[(last + 1) % n]) <= Vector2Distance(outline[first], outline[end])) continue;

        first = i;
        end = (last + 1) % n;
    }

    const Vector2 a = outline[first];
    const Vector2 b = outline[end];
    const float frontLength = Vector2Distance(a, b);

    Random random = { seed * 3266489917u | 1u };
    if (frontLength < 0.001f)
    {
        house.propCount = plantGarden(ground, lots[lot].box, random, house.props, 0);
        return;
    }

    // A curved stretch can bulge into the lot past the line from a to b, the front yard starts behind it
    const Vector2 axis = (b - a) / frontLength;
    const Vector2 inward = Vector2{ -axis.y, axis.x } * (signedArea(outline) >= 0.0f ? 1.0f : -1.0f);
    float bulge = 0.0f;
    for (int k = first; k != end; k = (k + 1) % n) bulge = fmaxf(bulge, Vector2DotProduct(outline[k] - a, inward));

    // A size drawn at random shrinks until the house fits somewhere along the front, its front yard keeps it away from the road
    Vector2 halfSize = { 0.5f * Clamp(frontLength * Lerp(0.45f, 0.7f, random.Next()), 4.5f, 14.0f), 0.0f };
    halfSize.y = 0.5f * Clamp(2.0f * halfSize.x * Lerp(0.75f, 1.3f, random.Next()), 4.5f, 14.0f);
    float yard = STREET_CLEARANCE + Lerp(1.5f, 5.0f, random.Next());
    const float preferred = random.Next() - 0.5f;
    house.roof = (int)(random.Next() * 1000.0f);

    // The house and its driveway take room like boxes, the props follow them. The place drawn at random goes first, then places spread
    // along the front
    std::array<Prop, MAX_HOUSE_PROPS + 2> placed;
    for (int attempt = 0; attempt < ATTEMPTS; ++attempt)
    {
        const float room = fmaxf(frontLength - 2.0f * halfSize.x, 0.0f);
        for (int k = 0; k < PLACES && !house.built; ++k)
        {
            const float shift = room * (k == 0 ? preferred : (float)(k - 1) / (float)(PLACES - 2) - 0.5f);
            placed[0] = { PropKind_Shed, Vector2Lerp(a, b, 0.5f) + axis * shift + inward * (bulge + yard + halfSize.y), halfSize, 0 };
            house.built = propFits(ground, axis, placed[0], {});
        }
        if (house.built) break;

        halfSize *= 0.88f;
        yard = fmaxf(STREET_CLEARANCE + 1.0f, 0.9f * yard);
    }

    if (!house.built)
    {
        house.propCount = plantGarden(ground, lots[lot].box, random, house.props, 0);
        return;
    }

    house.center = placed[0].position;
    house.axis = axis;
    house.halfSize = halfSize;
    int count = 1;

    // The driveway leaves the front next to a corner of the house straight to the side it reaches, which has to be a road. The part off
    // the road has to fit
    const Vector2 start = house.center - inward * halfSize.y + axis * ((random.Next() < 0.5f ? -1.0f : 1.0f) * fmaxf(halfSize.x - DRIVEWAY_HALF_WIDTH - 0.5f, 0.0f));

    float reach = FLT_MAX;
    int reached = -1;
    for (int i = 0; i < n; ++i)
    {
        float u = 0.0f;
        float v = 0.0f;
        if (!intersect(start, Vector2Negate(inward), outline[i], outline[(i + 1) % n] - outline[i], u, v) || u <= 0.0f || u >= reach || v < 0.0f || v > 1.0f) continue;

        reach = u;
        reached = i;
    }

    const float offRoad = 0.5f * (reach - STREET_CLEARANCE - 0.05f);
    if (reached >= 0 && streets[reached] && offRoad > 0.0f && propFits(ground, axis, { PropKind_Shed, start - inward * offRoad, { DRIVEWAY_HALF_WIDTH, offRoad }, 0 }, {}))
    {
        house.drivewayStart = start;
        house.drivewayEnd = start - inward * reach;
        placed[count++] = { PropKind_Shed, start - inward * (0.5f * reach), { DRIVEWAY_HALF_WIDTH, 0.5f * reach }, 0 };
    }

    // A car on a driveway long enough, then a pool and a shed now and then, trees and bushes where there is room
    const int firstProp = count;
    const std::span<Prop> room = std::span<Prop>(placed).first((size_t)(firstProp + MAX_HOUSE_PROPS));
    if (count == 2 && reach >= 5.6f && random.Next() < 0.6f) placed[count++] = { PropKind_Car, start - inward * 2.4f, { 0.9f, 2.0f }, (int)(random.Next() * 1000.0f) };
    const Box box = lots[lot].box;
    if (random.Next() < 0.35f) placeProp(ground, box, axis, { PropKind_Pool, {}, { Lerp(1.3f, 2.0f, random.Next()), Lerp(2.0f, 3.2f, random.Next()) }, 0 }, random, room, count);
    if (random.Next() < 0.3f) placeProp(ground, box, axis, { PropKind_Shed, {}, { Lerp(0.8f, 1.2f, random.Next()), Lerp(0.9f, 1.4f, random.Next()) }, 0 }, random, room, count);

    for (int k = 1 + (int)(4.0f * random.Next()); k > 0; --k) placeProp(ground, box, axis, { PropKind_Tree, {}, { Lerp(1.2f, 2.2f, random.Next()), 0.0f }, 0 }, random, room, count);
    for (int k = (int)(4.0f * random.Next()); k > 0; --k) placeProp(ground, box, axis, { PropKind_Bush, {}, { Lerp(0.5f, 0.8f, random.Next()), 0.0f }, 0 }, random, room, count);
    for (int k = firstProp; k < count; ++k) house.props[house.propCount++] = placed[k];
}

bool City::RoadThrough(std::span<const Vector2> polygon) const
{
    // How deep a road has to run into a polygon, roads along its sides are a little off
    constexpr float CLEARANCE = 0.6f;

    const Box box = boxOf(polygon);
    for (int r = 0; r < roadCount; ++r)
    {
        const Polyline &line = roads[r].line;
        for (int i = 0; i + 1 < line.count && overlap(box, roads[r].box); ++i)
        {
            const int steps = std::max(1, (int)ceilf(Vector2Distance(line.points[i], line.points[i + 1]) / (0.5f * CLEARANCE)));
            for (int k = 0; k <= steps; ++k)
            {
                const Vector2 p = Vector2Lerp(line.points[i], line.points[i + 1], (float)k / (float)steps);
                if (contains(box, p) && insidePolygon(polygon, p) && boundaryDistance(polygon, p) > CLEARANCE) return true;
            }
        }
    }
    return false;
}

// ---- City: drawing

void City::Draw(const Camera2D &camera) const
{
    constexpr Color LOT_COLOR = { 214, 222, 238, 64 };
    constexpr Color ROAD_COLOR = { 26, 29, 36, 255 };
    constexpr Color EDGE_COLOR = { 118, 130, 156, 255 };
    constexpr Color LINE_COLOR = { 150, 160, 182, 90 };

    // Muted colors for the dark theme, the parcel types are told apart by their hue
    constexpr Color STYLE_COLORS[] = { { 74, 88, 112, 255 }, { 66, 104, 90, 255 }, { 100, 86, 128, 255 }, { 124, 88, 84, 255 } };

    // In pixels on the screen except MAX_EDGE, the widest edge in the world
    constexpr float MIN_LOT_PIXELS = 3.0f;
    constexpr float EDGE_PIXELS = 1.3f;
    constexpr float MAX_EDGE = 0.35f;
    constexpr float MIN_EDGE_ROAD_PIXELS = 3.0f;
    constexpr float MIN_LINE_ROAD_PIXELS = 7.0f;
    constexpr float DASH_PIXELS = 7.0f;
    constexpr float GAP_PIXELS = 9.0f;
    constexpr float LINE_PIXELS = 1.2f;

    // The visible part of the world, the camera never rotates
    const Box view = { GetScreenToWorld2D({ 0.0f, 0.0f }, camera), GetScreenToWorld2D({ (float)GetScreenWidth(), (float)GetScreenHeight() }, camera) };
    const float worldPerPixel = 1.0f / camera.zoom;

    for (int i = 0; i < parcelCount; ++i)
    {
        if (overlap(parcels[i].box, view)) DrawParcel(i, STYLE_COLORS[(int)parcels[i].attributes.style]);
    }

    // Lots are thin outlines, skipped when they are too small to see
    for (int l = 0; l < lotCount; ++l)
    {
        const Box box = lots[l].box;
        if (!overlap(box, view) || fmaxf(box.max.x - box.min.x, box.max.y - box.min.y) < MIN_LOT_PIXELS * worldPerPixel) continue;

        const std::span<const Vector2> outline = LotOutline(l);
        DrawLineStrip(outline.data(), (int)outline.size(), LOT_COLOR);
        DrawLineV(outline.back(), outline.front(), LOT_COLOR);
    }

    // The roads cover the driveways out to their middle
    for (int l = 0; l < lotCount; ++l)
    {
        if (overlap(lots[l].box, view)) drawHouse(lots[l].house, worldPerPixel);
    }

    // Three passes merge intersections without seams: a lighter edge around every road, the roads, a dashed middle line on wide roads
    const float edge = fminf(EDGE_PIXELS * worldPerPixel, MAX_EDGE);
    std::array<Vector2, 2 * CURVE_POINTS> strip = {};
    for (int r = 0; r < roadCount; ++r)
    {
        if (roads[r].attributes.width < MIN_EDGE_ROAD_PIXELS * worldPerPixel || !overlap(roads[r].box, view)) continue;

        const int points = ribbon(roads[r].line, 0.5f * roads[r].attributes.width + edge, strip);
        DrawTriangleStrip(strip.data(), points, EDGE_COLOR);
    }
    DrawDisks(view, worldPerPixel, edge, 0.5f * MIN_EDGE_ROAD_PIXELS, EDGE_COLOR);

    for (int r = 0; r < roadCount; ++r)
    {
        if (overlap(roads[r].box, view)) DrawRoadHighlight(r, ROAD_COLOR);
    }
    DrawDisks(view, worldPerPixel, 0.0f, 1.0f, ROAD_COLOR);

    // The middle line stops before an intersection or a dead end, not at a node between just two roads
    const float gap = GAP_PIXELS * worldPerPixel;
    for (int r = 0; r < roadCount; ++r)
    {
        const Road &road = roads[r];
        if (road.attributes.width < MIN_LINE_ROAD_PIXELS * worldPerPixel || !overlap(road.box, view)) continue;

        const float startSkip = nodeDegree[road.from] == 2 ? 0.0f : nodeWidth[road.from] * 0.5f + gap * 0.5f;
        const float endSkip = nodeDegree[road.to] == 2 ? 0.0f : nodeWidth[road.to] * 0.5f + gap * 0.5f;
        drawDashes(road.line, startSkip, endSkip, DASH_PIXELS * worldPerPixel, gap, LINE_PIXELS * worldPerPixel, LINE_COLOR);
    }
}

void City::DrawRoadHighlight(int road, Color color) const
{
    std::array<Vector2, 2 * CURVE_POINTS> strip = {};
    const int points = ribbon(roads[road].line, 0.5f * roads[road].attributes.width, strip);
    DrawTriangleStrip(strip.data(), points, color);
}

void City::DrawMovedNode(int node, Vector2 position, Color color) const
{
    std::array<Polyline, MAX_MOVED_ROADS> lines;
    std::array<int, MAX_MOVED_ROADS> incident = {};
    float width = 0.0f;
    const int count = PreviewMovedNode(node, position, lines, incident, width);
    for (int i = 0; i < count; ++i) DrawSplineLinear(lines[i].points.data(), lines[i].count, width, color);
    if (count > 0) DrawCircleV(position, width * 0.5f, color);
}

void City::DrawSelection(const Selection &selection, Color color) const
{
    for (int i = 0; i < selection.parcelCount; ++i) DrawParcel(selection.parcels[i], color);
}

void City::DrawLot(int lot, Color color) const
{
    // A lot is cut into triangles only while it is shown, like the parcels they are given the other way round
    const std::span<const Vector2> outline = LotOutline(lot);
    std::array<uint16_t, 3 * MAX_CELL_POINTS> indices;
    const int count = triangulate(outline, indices);
    for (int t = 0; t + 2 < count; t += 3) DrawTriangle(outline[indices[t]], outline[indices[t + 2]], outline[indices[t + 1]], color);
}

void City::DrawParcel(int parcel, Color color) const
{
    // The triangles are counter-clockwise in the world like the outline, raylib culls them unless they are given the other way round
    const Parcel &p = parcels[parcel];
    const Vector2 *corner = parcelPoints.data() + p.firstPoint;
    for (int t = p.firstIndex; t + 2 < p.firstIndex + p.indexCount; t += 3)
    {
        DrawTriangle(corner[parcelIndices[t]], corner[parcelIndices[t + 2]], corner[parcelIndices[t + 1]], color);
    }
}

void City::DrawDisks(Box view, float worldPerPixel, float extra, float minRadiusPixels, Color color) const
{
    for (int n = 0; n < nodeCount; ++n)
    {
        const float radius = nodeWidth[n] * 0.5f;
        if (radius < minRadiusPixels * worldPerPixel || !overlap(inflate(Box{ nodes[n], nodes[n] }, radius + extra), view)) continue;

        DrawPoly(nodes[n], (int)Clamp(2.0f * radius / worldPerPixel, 8.0f, 36.0f), radius + extra, 0.0f, color);
    }
}

// A curved road from a to b whose middle passes through the point, a quadratic Bezier raised to a cubic one
static RoadAttributes curvedRoad(Vector2 a, Vector2 b, Vector2 through)
{
    const Vector2 control = through * 2.0f - (a + b) * 0.5f;
    RoadAttributes road;
    road.curved = true;
    road.controls = { Vector2Lerp(a, control, 2.0f / 3.0f), Vector2Lerp(b, control, 2.0f / 3.0f) };
    return road;
}

// Where a road would start or end: a ring on an intersection, a dot on a road, a cross where it stands free
static void drawAnchor(const RoadAnchor &anchor, float worldPerPixel, Color color)
{
    const float size = 6.0f * worldPerPixel;
    const Vector2 p = anchor.position;
    switch (anchor.kind)
    {
        case AnchorKind::Free:
        case AnchorKind::Node: DrawRing(p, size, size + 2.0f * worldPerPixel, 0.0f, 360.0f, 24, color); break;
        case AnchorKind::Road: DrawCircleV(p, 0.6f * size, color); break;
    }
}

Editor::Editor()
{
    for (int i = 0; i < Item_Count; ++i) icons[i] = LoadTexture(ITEMS[i].iconPath);
}

Editor::~Editor()
{
    for (Texture2D &icon : icons) UnloadTexture(icon);
}

void Editor::DrawWorld(City &city, const Camera2D &camera)
{
    // How close in pixels the mouse snaps to an intersection or a road, and how far a press and a release can be apart in a click
    constexpr float SNAP_PIXELS = 10.0f;
    constexpr float CLICK_PIXELS = 4.0f;

    const Tool tool = selectedButton == Item_None ? Tool::None : ITEMS[selectedButton].tool;
    const char *status = selectedButton == Item_None ? "Pick a tool or a parcel type" : ITEMS[selectedButton].hint;

    // Another tool or a right click drops what the tool was doing, dragging with the right button pans the camera and keeps it
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) rightPress = GetMousePosition();
    const bool rightClick = IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) && Vector2Distance(GetMousePosition(), rightPress) < CLICK_PIXELS;
    if (tool != activeTool || rightClick)
    {
        activeTool = tool;
        hasStart = false;
        hasEnd = false;
        movedNode = -1;
    }

    // The tools draw in the world below the toolbar, mouseOverToolbar is from the last frame here
    const Vector2 mouse = GetScreenToWorld2D(GetMousePosition(), camera);
    const float worldPerPixel = 1.0f / camera.zoom;
    const float snapRadius = SNAP_PIXELS * worldPerPixel;
    const bool click = !mouseOverToolbar && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    // What did not fit into the city stays in the status until the next click
    if (click) city.error = nullptr;

    switch (tool)
    {
        case Tool::None: break;
        case Tool::Road: UseRoad(city, mouse, snapRadius, worldPerPixel, click, status); break;
        case Tool::Remove: UseRemove(city, mouse, snapRadius, click); break;
        case Tool::Move: UseMove(city, mouse, snapRadius, worldPerPixel, click, status); break;
        case Tool::Parcel: UseParcel(city, mouse, click, status); break;
    }
    toolStatus = status;
}

void Editor::Draw(const City &city)
{
    constexpr float BUTTON_SIZE = 68.0f;
    constexpr float OPTION_WIDTH = 112.0f;
    constexpr float OPTION_HEIGHT = 30.0f;
    constexpr float GAP = 7.0f;
    constexpr int STATUS_FONT_SIZE = 20;

    const Tool tool = selectedButton == Item_None ? Tool::None : ITEMS[selectedButton].tool;

    // The buttons are centered at the bottom with an extra gap between tools, the options start above the first button
    float width = -GAP;
    for (int i = 0; i < Item_Count; ++i)
    {
        if (ITEMS[i].isOption) continue;
        width += BUTTON_SIZE + GAP;
        if (i > 0 && ITEMS[i].tool != ITEMS[i - 1].tool) width += GAP;
    }

    const float left = ((float)GetScreenWidth() - width) * 0.5f;
    const float buttonsY = (float)GetScreenHeight() - BUTTON_SIZE - 20.0f;
    const float optionsY = buttonsY - OPTION_HEIGHT - GAP;

    // Clicking the selected item unselects it, a button of another tool drops the option
    float buttonX = left;
    float optionX = left;
    mouseOverToolbar = false;
    for (int i = 0; i < Item_Count; ++i)
    {
        const ToolbarItem &item = ITEMS[i];
        if (item.isOption && item.tool != tool) continue;

        float &x = item.isOption ? optionX : buttonX;
        Item &selected = item.isOption ? selectedOption : selectedButton;
        if (!item.isOption && i > 0 && item.tool != ITEMS[i - 1].tool) x += GAP;

        const Rectangle bounds = item.isOption ? Rectangle{ x, optionsY, OPTION_WIDTH, OPTION_HEIGHT } : Rectangle{ x, buttonsY, BUTTON_SIZE, BUTTON_SIZE };
        if (DrawItem(bounds, icons[i], item.label, selected == i, !item.isOption))
        {
            if (!item.isOption && item.tool != tool) selectedOption = Item_None;
            selected = selected == i ? Item_None : (Item)i;
        }
        x += bounds.width + GAP;
    }

    // Status centered above the options, an error in red
    const char *status = city.error != nullptr ? city.error : toolStatus;
    const float textWidth = (float)MeasureText(status, STATUS_FONT_SIZE);
    const Rectangle box = { ((float)GetScreenWidth() - textWidth) * 0.5f - 10.0f, optionsY - GAP - 32.0f, textWidth + 24.0f, 32.0f };
    DrawRectangleRounded(box, 0.4f, 8, Color{ 20, 22, 28, 225 });
    DrawText(status, (int)box.x + 12, (int)box.y + 6, STATUS_FONT_SIZE, city.error != nullptr ? Color{ 240, 96, 96, 255 } : Color{ 226, 230, 238, 255 });
}

bool Editor::DrawItem(Rectangle bounds, Texture2D icon, const char *label, bool selected, bool iconAbove)
{
    constexpr int FONT_SIZE = 14;
    constexpr float PADDING = 4.0f;

    const bool hovered = CheckCollisionPointRec(GetMousePosition(), bounds);
    if (hovered) mouseOverToolbar = true;

    if (selected) DrawRectangleRounded(Rectangle{ bounds.x - 2.0f, bounds.y - 2.0f, bounds.width + 4.0f, bounds.height + 4.0f }, 0.2f, 8, Color{ 96, 165, 250, 255 });
    DrawRectangleRounded(bounds, 0.2f, 8, hovered ? Color{ 38, 42, 52, 245 } : Color{ 26, 29, 36, 240 });

    // Buttons have the icon above the label, options left of it
    const float iconSize = iconAbove ? bounds.height - FONT_SIZE - 3.0f * PADDING : bounds.height - 2.0f * PADDING;
    const Rectangle iconArea = { iconAbove ? bounds.x + (bounds.width - iconSize) * 0.5f : bounds.x + PADDING, bounds.y + PADDING, iconSize, iconSize };
    DrawTexturePro(icon, Rectangle{ 0.0f, 0.0f, (float)icon.width, (float)icon.height }, iconArea, Vector2{ 0.0f, 0.0f }, 0.0f, WHITE);

    const float textWidth = (float)MeasureText(label, FONT_SIZE);
    const float textX = iconAbove ? bounds.x + (bounds.width - textWidth) * 0.5f : iconArea.x + iconSize + PADDING;
    const float textY = iconAbove ? bounds.y + bounds.height - FONT_SIZE - PADDING : bounds.y + (bounds.height - FONT_SIZE) * 0.5f;
    DrawText(label, (int)textX, (int)textY, FONT_SIZE, selected ? Color{ 236, 240, 248, 255 } : Color{ 150, 158, 174, 255 });

    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void Editor::UseRoad(City &city, Vector2 mouse, float snapRadius, float worldPerPixel, bool click, const char *&status)
{
    const RoadAnchor anchor = city.Snap(mouse, snapRadius);
    const bool curved = selectedOption == Item_Curved;
    if (!hasEnd) drawAnchor(anchor, worldPerPixel, VALID_COLOR);
    if (!hasStart)
    {
        if (!click) return;

        roadStart = anchor;
        hasStart = true;
        return;
    }

    // A curved road gets its end first, then it bends through the mouse
    const RoadAnchor end = hasEnd ? roadEnd : anchor;
    const RoadAttributes road = hasEnd ? curvedRoad(roadStart.position, end.position, mouse) : RoadAttributes();
    bool alongRoad = false;
    const bool valid = city.CanAddRoad(roadStart, end, road, alongRoad);
    if (road.curved) DrawSplineSegmentBezierCubic(roadStart.position, road.controls[0], road.controls[1], end.position, road.width, valid ? VALID_COLOR : INVALID_COLOR);
    else DrawLineEx(roadStart.position, end.position, road.width, valid ? VALID_COLOR : INVALID_COLOR);

    if (curved && !hasEnd) status = "Click to place the end, then move the mouse to bend the road";
    else if (alongRoad) status = "Too close along another road";
    else if (!valid) status = "Too short, or it bends tighter than it is wide";
    else status = "Click to lay the road, right click to stop";

    if (!click) return;
    if (curved && !hasEnd)
    {
        roadEnd = anchor;
        hasEnd = true;
        return;
    }
    if (!valid || !city.AddRoad(roadStart, end, road)) return;

    // The next road goes on from the end of this one, found again since adding a road changes the indices
    roadStart = city.Snap(end.position, snapRadius);
    hasEnd = false;
}

void Editor::UseRemove(City &city, Vector2 mouse, float snapRadius, bool click)
{
    constexpr Color REMOVE_COLOR = { 240, 96, 96, 200 };

    int road = 0;
    if (!city.FindRoad(mouse, snapRadius, road)) return;

    city.DrawRoadHighlight(road, REMOVE_COLOR);
    if (click) city.RemoveRoadAt(mouse, snapRadius);
}

void Editor::UseMove(City &city, Vector2 mouse, float snapRadius, float worldPerPixel, bool click, const char *&status)
{
    // A release closer than this to where the intersection was is a click, nothing moves
    constexpr float CLICK_PIXELS = 4.0f;

    // The intersection under the mouse is picked up on a press and put down on the release
    if (movedNode < 0)
    {
        int node = 0;
        if (!city.FindNode(mouse, snapRadius, node)) return;

        drawAnchor(RoadAnchor{ AnchorKind::Node, node, 0, 0.0f, city.nodes[node] }, worldPerPixel, ANCHOR_COLOR);
        if (click) movedNode = node;
        return;
    }

    const bool valid = city.CanMoveNode(movedNode, mouse);
    city.DrawMovedNode(movedNode, mouse, valid ? VALID_COLOR : INVALID_COLOR);
    status = valid ? "Release to move the intersection" : "Its roads would cross, overlap or get too short there";
    if (!IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) return;

    if (valid && Vector2Distance(mouse, city.nodes[movedNode]) > CLICK_PIXELS * worldPerPixel) city.MoveNode(movedNode, mouse);
    movedNode = -1;
}

void Editor::UseParcel(City &city, Vector2 mouse, bool click, const char *&status)
{
    constexpr Color PREVIEW_COLOR = { 236, 240, 248, 60 };

    // With One Lot the lot under the mouse gets a new house drawn at random
    if (selectedOption == Item_OneLot)
    {
        status = "Click a lot to build a new house on it";
        int lot = 0;
        if (!city.LotAt(mouse, lot)) return;

        city.DrawLot(lot, PREVIEW_COLOR);
        if (click) city.BuildHouse(lot, city.lots[lot].house.seed + 1);
        return;
    }

    // The whole block under the mouse that becomes one parcel again, with One Parcel only the parcel under it
    const bool wholeBlock = selectedOption != Item_OneParcel;
    Selection selection;
    if (!city.FindBlock(mouse, wholeBlock, selection)) return;

    city.DrawSelection(selection, PREVIEW_COLOR);
    if (click) city.ApplyParcelTypeAt(mouse, (SubdivisionStyle)(selectedButton - Item_ParcelGrid), wholeBlock);
}
