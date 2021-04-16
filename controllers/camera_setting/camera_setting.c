#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"
#define SPEED 4
enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

int main(int argc, char **argv) {
    WbDeviceTag camera;
    int width, height;
//    int pause_counter = 0;
    int i, j;
    int red, blue, green;
    const char *color_names[3] = {"red", "green", "blue"};
    const char *ansi_colors[3] = {ANSI_COLOR_RED, ANSI_COLOR_GREEN, ANSI_COLOR_BLUE};
    const char *filenames[3] = {"red_blob.png", "green_blob.png", "blue_blob.png"};
    enum BLOB_TYPE current_blob;
    wb_robot_init();

    const int time_step = wb_robot_get_basic_time_step();
    /* Get the camera device, enable it, and store its width and height */
    camera = wb_robot_get_device("camera_sensor");
    wb_camera_enable(camera, time_step);
    width = wb_camera_get_width(camera);
    height = wb_camera_get_height(camera);
    int cnt =0;
    while (wb_robot_step(time_step) != -1) {
        cnt++;
        printf("cnt:%d\n",cnt);
        printf("できてる1\n");
        /* Get the new camera values */
        const unsigned char *image = wb_camera_get_image(camera); 
        /* Decrement the pause_counter */
        /* if (pause_counter > 0){ */
        /*   pause_counter--; */
        /* } else {  // pause_counter == 0 */
        /*   #<{(| Reset the sums |)}># */
        red = 0;
        green = 0;
        blue = 0;
        for (i = width / 3; i < 2 * width / 3; i++) {
            for (j = height / 2; j < 3 * height / 4; j++) {
//                printf("できてる22\n");
                red += wb_camera_image_get_red(image, width, i, j);
                blue += wb_camera_image_get_blue(image, width, i, j);
                green += wb_camera_image_get_green(image, width, i, j);
            }
        }
        printf("red:%d,green:%d,blue:%d\n",red,green,blue);
        printf("for文から出た\n");
        if ((red > 3 * green) && (red > 3 * blue))
            current_blob = RED;
        else if ((green > 3 * red) && (green > 3 * blue))
            current_blob = GREEN;
        else if ((blue > 3 * red) && (blue > 3 * green))
            current_blob = BLUE;
        else
            current_blob = NONE;

        printf("%d\n",current_blob);

        if(current_blob != NONE) {
            printf("Looks like I found a %s%s%s blob.\n", ansi_colors[current_blob], color_names[current_blob], ANSI_COLOR_RESET);
            // compute the file path in the user directory
            char *filepath;
#ifdef _WIN32
            const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
            filepath = (char *)malloc(strlen(user_directory) + 16);
            strcpy(filepath, user_directory);
            strcat(filepath, "\\");
#else
//            const char *user_directory = wbu_system_getenv("HOME");
            filepath = (char *)malloc(32);
            strcpy(filepath, "image");
            strcat(filepath, "/");
            printf("pass%s\n",filepath);
#endif
            printf("uuuuuuuuuuuu\n");
            strcat(filepath, filenames[current_blob]);
            wb_camera_save_image(camera, filepath, 100);
            printf("できてる????\n");
            free(filepath);
//            pause_counter = 1280 / time_step;
        }
        /* } */
        printf("ループしてくれ\n");
    };
    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
