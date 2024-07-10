#include <stdio.h>
#include <math.h>

int main() {
    int input_value = 700;
    double converted_value;

    // 対数変換
    converted_value = log(input_value - 499); // 500からの距離を取得し、対数変換を行う

    // スケーリング
    converted_value *= 100.0 / log(800 - 499); // 変換後の値の最大値が100になるようにスケーリングする

    printf("Input value: %d, Converted value: %.2f\n", input_value, converted_value);

    return 0;
}
