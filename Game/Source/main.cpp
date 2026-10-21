#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include "City.h"


// Shows the box in the middle of the window above the toolbar
static void fitCamera(Box box, Camera2D &camera)
{
    constexpr float FIT_MARGIN = 1.2f;
    constexpr float TOOLBAR_RESERVED = 150.0f;

    const float usableHeight = (float)GetScreenHeight() - TOOLBAR_RESERVED;
    const Vector2 size = box.max - box.min;
    camera.target = (box.min + box.max) * 0.5f;
    camera.offset = Vector2{ GetScreenWidth() * 0.5f, usableHeight * 0.5f };
    camera.rotation = 0.0f;
    camera.zoom = fminf((float)GetScreenWidth() / (fmaxf(size.x, 1.0f) * FIT_MARGIN), usableHeight / (fmaxf(size.y, 1.0f) * FIT_MARGIN));
}

static void updateCamera(Camera2D &camera)
{
    constexpr float ZOOM_SPEED = 0.1f;
    constexpr float ZOOM_MIN = 0.01f;
    constexpr float ZOOM_MAX = 200.0f;

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

// The lot and block sizes of every parcel type, a parcel is generated with the settings of its type at that time
static void configureParcels(City &city)
{
    city.parcelSettings[SubdivisionStyle_Grid] = { .minParcelArea = 150.0f, .maxParcelArea = 550.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[SubdivisionStyle_Organic] = { .minParcelArea = 100.0f, .maxParcelArea = 500.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[SubdivisionStyle_Radial] = { .minParcelArea = 150.0f, .maxParcelArea = 400.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.0f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };
    city.parcelSettings[SubdivisionStyle_Random] = { .minParcelArea = 50.0f, .maxParcelArea = 800.0f, .minParcelWidth = 5.0f, .maxParcelWidth = 20.0f, .splitIrregularity = 0.2f, .minBlockSize = 25.0f, .maxBlockSize = 125.0f };

    // The type of the parcels that roads enclose for the first time
    city.newParcelStyle = SubdivisionStyle_Grid;
}

static void runApp()
{
    constexpr Color HINT_COLOR = Color{ 120, 128, 142, 255 };
    constexpr Color BACKGROUND_COLOR = Color{ 17, 19, 24, 255 };

    auto city = std::make_unique<City>();
    if (city == nullptr) return;

    configureParcels(*city);
    Editor editor;
    Camera2D camera = {};
    fitCamera(city->Bounds(), camera);

    while (!WindowShouldClose())
    {
        updateCamera(camera);
        if (IsKeyPressed(KEY_R)) fitCamera(city->Bounds(), camera);

        BeginDrawing();
        ClearBackground(BACKGROUND_COLOR);

        BeginMode2D(camera);
        city->Draw(camera);
        editor.DrawWorld(*city, camera);
        EndMode2D();

        editor.Draw(*city);

        DrawText("RMB/MMB drag: pan   Wheel: zoom   R: fit", 10, 10, 20, HINT_COLOR);
        DrawText(TextFormat("%d FPS", GetFPS()), 10, 40, 20, HINT_COLOR);

        EndDrawing();
    }
}

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, GAME_NAME);
    SetTargetFPS(60);

    rlDisableBackfaceCulling();

    runApp();

    CloseWindow();
    return 0;
}
