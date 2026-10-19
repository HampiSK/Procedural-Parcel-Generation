#include <array>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include "City.h"

// Frames the box in the window, above the toolbar while it is shown
static void fitCamera(Box box, bool aboveToolbar, Camera2D &camera)
{
    constexpr float FIT_MARGIN = 1.2f;
    constexpr float TOOLBAR_RESERVED = 150.0f;

    const float usableHeight = (float)GetScreenHeight() - (aboveToolbar ? TOOLBAR_RESERVED : 0.0f);
    const Vector2 size = box.max - box.min;
    camera.target = (box.min + box.max) * 0.5f;
    camera.offset = Vector2{ GetScreenWidth() * 0.5f, usableHeight * 0.5f };
    camera.rotation = 0.0f;
    camera.zoom = fminf((float)GetScreenWidth() / (fmaxf(size.x, 1.0f) * FIT_MARGIN), usableHeight / (fmaxf(size.y, 1.0f) * FIT_MARGIN));
}

// The lot and block sizes of every parcel type, a parcel is generated with the settings of its type at that time
static void configureParcels(City &city)
{
    city.parcelSettings[(size_t)SubdivisionStyle::Grid] = { .minParcelArea = 100.0f, .maxParcelArea = 500.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[(size_t)SubdivisionStyle::Organic] = { .minParcelArea = 100.0f, .maxParcelArea = 500.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[(size_t)SubdivisionStyle::Radial] = { .minParcelArea = 100.0f, .maxParcelArea = 500.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[(size_t)SubdivisionStyle::Random] = { .minParcelArea = 100.0f, .maxParcelArea = 500.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };

    // The type of the parcels that roads enclose for the first time
    city.newParcelStyle = SubdivisionStyle::Grid;
}

static void updateCamera(Camera2D &camera)
{
    constexpr float ZOOM_SPEED = 0.1f;
    constexpr float ZOOM_MIN = 0.01f;
    constexpr float ZOOM_MAX = 200.0f;

    // Pan with right or middle mouse button
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) camera.target -= GetMouseDelta() / camera.zoom;

    // Zoom towards the cursor, the world point under it stays put
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f)
    {
        const Vector2 mouse = GetMousePosition();
        camera.target = GetScreenToWorld2D(mouse, camera);
        camera.offset = mouse;
        camera.zoom = Clamp(camera.zoom * expf(wheel * ZOOM_SPEED), ZOOM_MIN, ZOOM_MAX);
    }
}

// ---- Intro, writes PROCEDURAL PARCEL GENERATION into the city with roads in ten seconds

// A letter as closed outlines of roads in the order a pen draws them, in units of a glyph 10 high with its top at 0. M starts an outline,
// L is a straight road to a point and Q a road curved over a control point to a point. The corner at the end of l or q stays sharp when
// the letters are rounded off. Holes are left open, so that every outline is one parcel
struct Glyph
{
    char letter = ' ';
    float width = 0.0f;
    const char *path = "";
};

// A road of the title, from is how far along the path of its letter it starts
struct IntroStroke
{
    Vector2 a = { 0.0f, 0.0f };
    Vector2 b = { 0.0f, 0.0f };
    Vector2 control = { 0.0f, 0.0f };
    bool curved = false;
    bool startsOutline = false;
    bool closesOutline = false;
    bool sharpEnd = false;
    float from = 0.0f;
    float length = 0.0f;
};

// A corner of a letter that moves from where it was drawn to where its roads make a round corner, at is where it is now
struct IntroCorner
{
    Vector2 from = { 0.0f, 0.0f };
    Vector2 to = { 0.0f, 0.0f };
    Vector2 at = { 0.0f, 0.0f };
};

// A letter of the title, its strokes, its corners and a point inside each of its outlines are ranges in the arrays of the intro
struct IntroLetter
{
    SubdivisionStyle style = SubdivisionStyle::Grid;
    size_t firstStroke = 0;
    size_t strokeCount = 0;
    size_t firstCorner = 0;
    size_t cornerCount = 0;
    size_t firstProbe = 0;
    size_t probeCount = 0;
    float length = 0.0f;

    // When its pen starts and its corners start to move, and how far it got
    float start = 0.0f;
    float roundTime = 0.0f;
    size_t added = 0;
    bool rounded = false;
};

// The letters of the title with their strokes, corners and points one after another, and the time since the intro started
struct Intro
{
    std::vector<IntroStroke> strokes;
    std::vector<IntroCorner> corners;
    std::vector<Vector2> probes;
    std::vector<IntroLetter> letters;
    Box box;
    float time = 0.0f;

    // The type of new parcels in the editor, the intro uses the types of its letters while it plays
    SubdivisionStyle newParcelStyle = SubdivisionStyle::Grid;
};

// Seconds a letter takes to write and its corners to round off, and the length of the intro
static constexpr float INTRO_WRITE = 1.0f;
static constexpr float INTRO_ROUND = 0.35f;
static constexpr float INTRO_LENGTH = 10.0f;

static constexpr Glyph GLYPHS[] = {
    { 'A', 10.0f, "M 0 10 L 2.25 0 L 4.5 0 l 3.2 5.8 l 6.8 5.8 L 5.5 0 L 7.75 0 L 10 10 L 7.75 10 L 7.26 7.8 L 2.74 7.8 L 2.25 10 L 0 10" },
    { 'C', 7.6f, "M 7.6 0 L 3.8 0 Q 0 0 0 5 Q 0 10 3.8 10 L 7.6 10 L 7.6 7.8 L 3.8 7.8 Q 2.2 7.8 2.2 5 Q 2.2 2.2 3.8 2.2 L 7.6 2.2 L 7.6 0" },
    { 'D', 7.8f, "M 0 0 L 3.3 0 Q 7.8 0 7.8 5 Q 7.8 10 3.3 10 L 3.3 7.8 Q 5.6 7.8 5.6 5 Q 5.6 2.2 3.3 2.2 L 2.2 2.2 L 2.2 10 L 0 10 L 0 0" },
    { 'E', 6.4f, "M 0 0 L 6.4 0 L 6.4 2.2 L 2.2 2.2 L 2.2 3.9 L 5.6 3.9 L 5.6 6.1 L 2.2 6.1 L 2.2 7.8 L 6.4 7.8 L 6.4 10 L 0 10 L 0 0" },
    { 'G', 8.0f, "M 8 0 L 4 0 Q 0 0 0 5 Q 0 10 4 10 L 8 10 L 8 4.6 L 4.6 4.6 L 4.6 6.6 L 5.8 6.6 L 5.8 7.8 L 4 7.8 Q 2.2 7.8 2.2 5 Q 2.2 2.2 4 2.2 L 8 2.2 L 8 0" },
    { 'I', 2.2f, "M 0 0 L 2.2 0 L 2.2 10 L 0 10 L 0 0" },
    { 'L', 6.2f, "M 0 0 L 2.2 0 L 2.2 7.8 L 6.2 7.8 L 6.2 10 L 0 10 L 0 0" },
    { 'N', 7.8f, "M 0 0 L 2.4 0 L 5.6 5.6 L 5.6 0 L 7.8 0 L 7.8 10 L 5.4 10 L 2.2 4.4 L 2.2 10 L 0 10 L 0 0" },
    { 'O', 8.6f, "M 3.8 0 Q 0 0 0 5 q 0 10 3.8 10 l 3.8 7.8 Q 2.2 7.8 2.2 5 q 2.2 2.2 3.8 2.2 l 3.8 0 M 4.8 0 l 4.8 2.2 Q 6.4 2.2 6.4 5 q 6.4 7.8 4.8 7.8 l 4.8 10 Q 8.6 10 8.6 5 q 8.6 0 4.8 0" },
    { 'P', 7.2f, "M 0 0 L 4.4 0 Q 7.2 0 7.2 3.3 Q 7.2 6.6 4.4 6.6 L 3.3 6.6 L 3.3 4.4 L 4.4 4.4 Q 5 4.4 5 3.3 Q 5 2.2 4.4 2.2 L 2.2 2.2 L 2.2 10 L 0 10 L 0 0" },
    { 'R', 7.8f, "M 0 0 L 4.4 0 Q 7.2 0 7.2 3.3 q 7.2 6.6 6.1 6.6 L 7.8 10 L 5.3 10 L 3.3 6.6 L 3.3 4.4 L 4.4 4.4 Q 5 4.4 5 3.3 Q 5 2.2 4.4 2.2 L 2.2 2.2 L 2.2 10 L 0 10 L 0 0" },
    { 'T', 7.4f, "M 0 0 L 7.4 0 L 7.4 2.2 L 4.8 2.2 L 4.8 10 L 2.6 10 L 2.6 2.2 L 0 2.2 L 0 0" },
    { 'U', 7.6f, "M 0 0 L 2.2 0 L 2.2 6.2 Q 2.2 7.8 3.8 7.8 Q 5.4 7.8 5.4 6.2 L 5.4 0 L 7.6 0 L 7.6 6.2 Q 7.6 10 3.8 10 Q 0 10 0 6.2 L 0 0" },
};

// Every letter of the title has a glyph
static const Glyph &glyphOf(char letter)
{
    for (const Glyph &glyph : GLYPHS)
    {
        if (glyph.letter == letter) return glyph;
    }
    return GLYPHS[0];
}

// The point at t along a stroke, a curved stroke is a quadratic Bezier
static Vector2 strokePoint(const IntroStroke &stroke, float t)
{
    if (!stroke.curved) return Vector2Lerp(stroke.a, stroke.b, t);
    return Vector2Lerp(Vector2Lerp(stroke.a, stroke.control, t), Vector2Lerp(stroke.control, stroke.b, t), t);
}

// The part of a stroke from t0 to t1, the control point of a part of a quadratic Bezier is its blossom at t0 and t1
static IntroStroke strokePart(const IntroStroke &stroke, float t0, float t1)
{
    constexpr int LENGTH_PIECES = 16;

    IntroStroke part = stroke;
    part.a = strokePoint(stroke, t0);
    part.b = strokePoint(stroke, t1);
    part.control = stroke.a * ((1.0f - t0) * (1.0f - t1)) + stroke.control * (t0 + t1 - 2.0f * t0 * t1) + stroke.b * (t0 * t1);
    part.startsOutline = stroke.startsOutline && t0 == 0.0f;
    part.length = 0.0f;
    for (int i = 0; i < LENGTH_PIECES; i++) part.length += Vector2Distance(strokePoint(part, (float)i / LENGTH_PIECES), strokePoint(part, (float)(i + 1) / LENGTH_PIECES));
    return part;
}

// Draws a stroke from its start to t as a line of the width
static void drawStroke(const IntroStroke &stroke, float t, float width, Color color)
{
    if (stroke.curved) DrawSplineSegmentBezierQuadratic(stroke.a, Vector2Lerp(stroke.a, stroke.control, t), strokePoint(stroke, t), width, color);
    else DrawLineEx(stroke.a, strokePoint(stroke, t), width, color);
}

// How far along its path the pen of a letter is, it speeds up and slows down
static float penAt(const IntroLetter &letter, float time)
{
    const float t = Clamp((time - letter.start) / INTRO_WRITE, 0.0f, 1.0f);
    return letter.length * t * t * (3.0f - 2.0f * t);
}

static void addStroke(Intro &intro, IntroLetter &letter, IntroStroke stroke)
{
    stroke.from = letter.length;
    letter.length += stroke.length;
    intro.strokes.push_back(stroke);
}

// Turns a glyph into the strokes of a letter at origin. A sharp corner of an outline gets intersections on its sides, moving it to the
// middle of the arc that touches its sides there makes it a round corner later. Every outline gets a point inside it
static void addLetter(Intro &intro, const Glyph &glyph, Vector2 origin, float scale, IntroLetter letter)
{
    // The most strokes of a glyph and the thickness of its strokes. A corner is sharp while the cosine of the angle between its sides is
    // above SHARP, it is rounded over ROUND_SIZE of its sides and stays ROUND_CLEARANCE away from the intersections there, roads keep
    // that much room. A part between two rounded ones that would be shorter than MIN_PART goes to them
    constexpr size_t MAX_STROKES = 24;
    constexpr float STROKE = 2.2f;
    constexpr float SHARP = -0.9f;
    constexpr float ROUND_SIZE = 5.5f;
    constexpr float ROUND_CLEARANCE = 2.6f;
    constexpr float MIN_PART = 2.0f;

    std::array<IntroStroke, MAX_STROKES> path = {};
    size_t pathCount = 0;
    Vector2 pen = origin;
    bool startsOutline = false;
    for (const char *p = glyph.path; *p != '\0' && pathCount < MAX_STROKES;)
    {
        const char command = *p++;
        if (command == ' ') continue;

        float values[4] = {};
        const bool curved = command == 'Q' || command == 'q';
        const int count = curved ? 4 : 2;
        for (int k = 0; k < count; k++)
        {
            char *end = nullptr;
            values[k] = strtof(p, &end);
            p = end;
        }
        const Vector2 point = origin + Vector2{ values[count - 2], values[count - 1] } *scale;
        if (command == 'M')
        {
            pen = point;
            startsOutline = true;
            continue;
        }

        IntroStroke stroke;
        stroke.a = pen;
        stroke.b = point;
        stroke.control = origin + Vector2{ values[0], values[1] } *scale;
        stroke.curved = curved;
        stroke.startsOutline = startsOutline;
        stroke.sharpEnd = command == 'l' || command == 'q';
        path[pathCount++] = strokePart(stroke, 0.0f, 1.0f);
        pen = point;
        startsOutline = false;
    }

    letter.firstStroke = intro.strokes.size();
    letter.firstCorner = intro.corners.size();
    letter.firstProbe = intro.probes.size();
    std::array<float, MAX_STROKES> rounds = {};
    for (size_t s = 0; s < pathCount; s++)
    {
        if (!path[s].startsOutline) continue;

        // An outline runs up to the next one, the direction of a curved stroke at an end is its tangent
        size_t end = s + 1;
        while (end < pathCount && !path[end].startsOutline) end++;
        float area = 0.0f;
        for (size_t k = s; k < end; k++)
        {
            const IntroStroke &previous = path[k == s ? end - 1 : k - 1];
            const IntroStroke &next = path[k];
            const Vector2 toPrevious = Vector2Normalize((previous.curved ? previous.control : previous.a) - next.a);
            const Vector2 toNext = Vector2Normalize((next.curved ? next.control : next.b) - next.a);
            const float cosine = Vector2DotProduct(toPrevious, toNext);
            area += Vector2CrossProduct(next.a, next.b);
            rounds[k] = 0.0f;
            if (cosine <= SHARP || previous.sharpEnd) continue;

            // The middle of the arc is on the bisector, the parts at the corner are about the arc then. A sharp corner stops before it
            // gets too close to where the arc starts
            rounds[k] = fminf(ROUND_SIZE, 0.5f * fminf(previous.length, next.length));
            const float sine = sqrtf(0.5f - 0.5f * cosine);
            const float across = rounds[k] * sine;
            float depth = rounds[k] * (1.0f - sine) / sqrtf(0.5f + 0.5f * cosine);
            if (across < ROUND_CLEARANCE) depth = fminf(depth, rounds[k] * sqrtf(0.5f + 0.5f * cosine) - sqrtf(ROUND_CLEARANCE * ROUND_CLEARANCE - across * across));
            if (depth > 0.0f) intro.corners.push_back({ next.a, next.a + Vector2Normalize(toPrevious + toNext) * depth, next.a });
        }

        // Every stroke splits into the parts at its rounded ends and the part between them
        for (size_t k = s; k < end; k++)
        {
            const float rounded[2] = { rounds[k], rounds[k + 1 < end ? k + 1 : s] };
            float t0 = rounded[0] / path[k].length;
            float t1 = 1.0f - rounded[1] / path[k].length;
            if ((t1 - t0) * path[k].length < MIN_PART)
            {
                t0 = rounded[0] > 0.0f && rounded[1] > 0.0f ? 0.5f * (t0 + t1) : rounded[0] > 0.0f ? 1.0f : 0.0f;
                t1 = t0;
            }
            if (t0 > 0.0f) addStroke(intro, letter, strokePart(path[k], 0.0f, t0));
            if (t1 > t0) addStroke(intro, letter, strokePart(path[k], t0, t1));
            if (t1 < 1.0f) addStroke(intro, letter, strokePart(path[k], t1, 1.0f));
        }
        intro.strokes.back().closesOutline = true;

        // Half a stroke in from the middle of the first side, the inside is on the left when the outline turns counterclockwise
        const Vector2 along = Vector2Normalize(path[s].b - path[s].a);
        const Vector2 inward = area > 0.0f ? Vector2{ -along.y, along.x } : Vector2{ along.y, -along.x };
        intro.probes.push_back(strokePoint(path[s], 0.5f) + inward * (0.5f * STROKE * scale));
    }

    letter.strokeCount = intro.strokes.size() - letter.firstStroke;
    letter.cornerCount = intro.corners.size() - letter.firstCorner;
    letter.probeCount = intro.probes.size() - letter.firstProbe;
    intro.letters.push_back(letter);
    intro.box.min = { fminf(intro.box.min.x, origin.x), fminf(intro.box.min.y, origin.y) };
    intro.box.max = { fmaxf(intro.box.max.x, origin.x + glyph.width * scale), fmaxf(intro.box.max.y, origin.y + 10.0f * scale) };
}

// Lays the title out in lines around the origin. The letters are written one after another, every one with another parcel type than the
// one before, and the rounding goes over them as a wave in the last second
static void buildIntro(const City &city, Intro &intro)
{
    constexpr const char *LINES[] = { "PROCEDURAL", "PARCEL", "GENERATION" };
    constexpr float SCALE = 5.0f;
    constexpr float SPACING = 1.6f;
    constexpr float LEADING = 13.0f;

    // Seconds until the first letter starts and between two letters, and when the rounding starts and between two letters
    constexpr float START = 0.2f;
    constexpr float STAGGER = 0.31f;
    constexpr float ROUND_START = 8.95f;
    constexpr float ROUND_STAGGER = 0.025f;

    intro.newParcelStyle = city.newParcelStyle;
    for (size_t line = 0; line < std::size(LINES); line++)
    {
        float width = -SPACING;
        for (const char *c = LINES[line]; *c != '\0'; c++) width += glyphOf(*c).width + SPACING;

        float x = -0.5f * width;
        const float y = LEADING * ((float)line - 0.5f * (float)(std::size(LINES) - 1)) - 5.0f;
        for (const char *c = LINES[line]; *c != '\0'; c++)
        {
            const float index = (float)intro.letters.size();
            IntroLetter letter;
            letter.style = (SubdivisionStyle)(intro.letters.size() % SUBDIVISION_STYLE_COUNT);
            letter.start = START + STAGGER * index;
            letter.roundTime = ROUND_START + ROUND_STAGGER * index;
            addLetter(intro, glyphOf(*c), Vector2{ x, y } *SCALE, SCALE, letter);
            x += glyphOf(*c).width + SPACING;
        }
    }
}

// Plays delta seconds of the intro, false once it is over. A road is added when the pen of its letter got to its end, a road that closes
// an outline makes a parcel of the type of the letter. At the end the corners of the letters move to round them off, then each letter
// gets new lots. The intro removes no road
static bool playIntro(City &city, Intro &intro, float delta)
{
    constexpr float SNAP = 0.5f;

    intro.time += delta;
    const float time = intro.time;
    city.autoGenerate = true;
    for (IntroLetter &letter : intro.letters)
    {
        const float drawn = penAt(letter, time);
        for (; letter.added < letter.strokeCount; letter.added++)
        {
            const IntroStroke &stroke = intro.strokes[letter.firstStroke + letter.added];
            if (stroke.from + stroke.length > drawn) break;

            // A road that closes no outline leaves the parcels as they are, so it goes to the graph alone and the city builds its parcels
            // again only for the roads that change them
            RoadAttributes road;
            road.curved = stroke.curved;
            road.controls = { Vector2Lerp(stroke.a, stroke.control, 2.0f / 3.0f), Vector2Lerp(stroke.b, stroke.control, 2.0f / 3.0f) };
            const RoadAnchor a = city.graph.Snap(stroke.a, SNAP);
            const RoadAnchor b = city.graph.Snap(stroke.b, SNAP);
            bool alongRoad = false;
            city.newParcelStyle = letter.style;
            if (stroke.closesOutline) city.AddRoad(a, b, road);
            else if (city.CanAddRoad(a, b, road, alongRoad)) city.graph.AddRoad(a, b, road);
        }
    }

    // The corners move without generating and the parcels follow them once a frame. Moving the last node that moved to where it is builds
    // them again, it can move there as nothing changed since
    city.autoGenerate = false;
    size_t moved = SIZE_MAX;
    for (IntroLetter &letter : intro.letters)
    {
        if (letter.rounded || time <= letter.roundTime) continue;

        const float t = fminf((time - letter.roundTime) / INTRO_ROUND, 1.0f);
        for (size_t c = letter.firstCorner; c < letter.firstCorner + letter.cornerCount; c++)
        {
            IntroCorner &corner = intro.corners[c];
            const Vector2 position = Vector2Lerp(corner.from, corner.to, t * t * (3.0f - 2.0f * t));
            size_t node = 0;
            if (!city.graph.FindNode(corner.at, 0.01f, node) || !city.graph.MoveNode(node, position)) continue;

            corner.at = position;
            moved = node;
        }
    }
    if (moved != SIZE_MAX) city.MoveNode(moved, city.graph.nodes[moved]);

    // A letter whose corners arrived gets new lots for its new shape
    city.autoGenerate = true;
    for (IntroLetter &letter : intro.letters)
    {
        if (letter.rounded || time < letter.roundTime + INTRO_ROUND) continue;

        for (size_t p = letter.firstProbe; p < letter.firstProbe + letter.probeCount; p++) city.ApplyParcelTypeAt(intro.probes[p], letter.style, true);
        letter.rounded = true;
    }

    if (time < INTRO_LENGTH) return true;
    city.newParcelStyle = intro.newParcelStyle;
    return false;
}

// The roads the pens are on with the pens. The light edges go under the city, so that the roads they reach cover them like the roads of
// the city cover each other, the rest goes over it
static void drawIntro(const Intro &intro, const Camera2D &camera, bool over)
{
    // The colors and the edge of RoadGraph::Draw, and the pens in the color of the markers of the editor
    constexpr Color ROAD_COLOR = { 26, 29, 36, 255 };
    constexpr Color EDGE_COLOR = { 118, 130, 156, 255 };
    constexpr float EDGE_PIXELS = 1.3f;
    constexpr float MAX_EDGE = 0.35f;
    constexpr Color PEN_COLOR = { 96, 165, 250, 255 };
    constexpr float PEN_PIXELS = 4.0f;

    const float width = RoadAttributes().width;
    const float edge = over ? 0.0f : fminf(EDGE_PIXELS / camera.zoom, MAX_EDGE);
    const Color color = over ? ROAD_COLOR : EDGE_COLOR;
    for (const IntroLetter &letter : intro.letters)
    {
        const float drawn = penAt(letter, intro.time);
        if (letter.added == letter.strokeCount || drawn <= 0.0f) continue;

        // Round ends like a dead end, a road that goes on from another one starts at the end of that one
        const IntroStroke &stroke = intro.strokes[letter.firstStroke + letter.added];
        const float t = Clamp((drawn - stroke.from) / stroke.length, 0.0f, 1.0f);
        const Vector2 pen = strokePoint(stroke, t);
        drawStroke(stroke, t, width + 2.0f * edge, color);
        DrawCircleV(pen, 0.5f * width + edge, color);
        if (stroke.startsOutline) DrawCircleV(stroke.a, 0.5f * width + edge, color);
        if (over) DrawCircleV(pen, 2.0f * PEN_PIXELS / camera.zoom, Fade(PEN_COLOR, 0.25f));
        if (over) DrawCircleV(pen, PEN_PIXELS / camera.zoom, PEN_COLOR);
    }
}

int main()
{
    // A longer frame, like the first one, does not skip a part of the intro
    constexpr float MAX_FRAME = 1.0f / 30.0f;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, GAME_NAME);
    SetTargetFPS(60);

    // Nothing in this app needs culling and 2D triangles must be counter-clockwise otherwise
    rlDisableBackfaceCulling();

    {
        // Scoped so the textures are unloaded before CloseWindow destroys the GL context
        City city;
        configureParcels(city);
        Editor editor;

        // The intro writes the title into the city first and shows the city alone, space skips to its end
        Intro intro;
        buildIntro(city, intro);
        bool playing = true;

        Camera2D camera = {};
        fitCamera(intro.box, false, camera);

        while (!WindowShouldClose())
        {
            if (playing) playing = playIntro(city, intro, IsKeyPressed(KEY_SPACE) ? INTRO_LENGTH : fminf(GetFrameTime(), MAX_FRAME));
            updateCamera(camera);
            if (IsKeyPressed(KEY_R)) fitCamera(city.graph.Bounds(), !playing, camera);

            BeginDrawing();
            ClearBackground(Color{ 17, 19, 24, 255 });

            BeginMode2D(camera);
            if (playing) drawIntro(intro, camera, false);
            city.Draw(camera);
            if (playing) drawIntro(intro, camera, true);
            else editor.DrawWorld(city, camera);
            EndMode2D();

            if (!playing)
            {
                editor.Draw();

                const Color hintColor = Color{ 120, 128, 142, 255 };
                DrawText("RMB/MMB drag: pan   Wheel: zoom   R: fit", 10, 10, 20, hintColor);
                DrawText(TextFormat("%d FPS", GetFPS()), 10, 40, 20, hintColor);
            }
            EndDrawing();
        }
    }

    CloseWindow();
    return 0;
}