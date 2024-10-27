#include <Novice.h>
#include <cassert>
#include <cmath>

const char kWindowTitle[] = "LE2C_27_リ_ヨン_MT4";

struct Vector3 {
	float x, y, z;
};

struct Matrix4x4 {
	float m[4][4];
};

float Dot(const Vector3& v1, const Vector3& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
float Length(const Vector3& v) { return std::sqrt(Dot(v, v)); }


Vector3 Normalize(const Vector3& v) {
	float length = Length(v);
	assert(length != 0.0f);
	return { v.x / length, v.y / length, v.z / length };
}

Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angle) {

	 Vector3 n = Normalize(axis);
	//資料p20を参考に中身を埋める。nはaxisのこと
	float cos = std::cos(angle);
	float sin = std::sin(angle);
	float MinusCos = 1.0f - cos;

	Matrix4x4 rotateMatrix = {};
    rotateMatrix.m[0][0] = cos + n.x * n.x * MinusCos;
    rotateMatrix.m[0][1] = n.x * n.y * MinusCos + n.z * sin;
    rotateMatrix.m[0][2] = n.x * n.z * MinusCos - n.y * sin;
    rotateMatrix.m[0][3] = 0.0f;

    rotateMatrix.m[1][0] = n.y * n.x * MinusCos - n.z * sin;
    rotateMatrix.m[1][1] = cos + n.y * n.y * MinusCos;
    rotateMatrix.m[1][2] = n.y * n.z * MinusCos + n.x * sin;
    rotateMatrix.m[1][3] = 0.0f;

    rotateMatrix.m[2][0] = n.z * n.x * MinusCos + n.y * sin;
    rotateMatrix.m[2][1] = n.z * n.y * MinusCos - n.x * sin;
    rotateMatrix.m[2][2] = cos + n.z * n.z * MinusCos;
    rotateMatrix.m[2][3] = 0.0f;

    rotateMatrix.m[3][0] = 0.0f;
    rotateMatrix.m[3][1] = 0.0f;
    rotateMatrix.m[3][2] = 0.0f;
    rotateMatrix.m[3][3] = 1.0f;

	return rotateMatrix;
}

static const int kRowHeight = 20;
static const int kColumnWidth = 60;
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(
				x + column * kColumnWidth, y + (row + 1) * kRowHeight, "%6.03f",
				matrix.m[row][column]);
		}
	}
}


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	const int kWindowWidth = 1280;
	const int kWindowHeight = 720;
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		Vector3 axis = Normalize(Vector3{ 1.0f, 1.0f, 1.0f });
		float angle = 0.44f;
		Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrintf(0, 0, rotateMatrix, "rotateMatrix");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}