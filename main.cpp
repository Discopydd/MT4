#include <Novice.h>
#include <cmath>

const char kWindowTitle[] = "MT4_01_05";

struct Vector3 {
	float x, y, z;
};

struct Quaternion {
	float x, y, z, w;
};

struct Matrix4x4 {
	float m[4][4];
};

Vector3 Normalize(const Vector3& v) {
	 float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length != 0) {
        return { v.x / length, v.y / length, v.z / length };
    }
    return { 0.0f, 0.0f, 0.0f };
}

Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	 Vector3 result;
    result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0];
    result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1];
    result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2];
    return result;
}


Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle) {
	 Vector3 normalizedAxis = Normalize(axis);
    float halfAngle = angle * 0.5f;
    float sinHalfAngle = std::sin(halfAngle);
    return {
        normalizedAxis.x * sinHalfAngle,
        normalizedAxis.y * sinHalfAngle,
        normalizedAxis.z * sinHalfAngle,
        std::cos(halfAngle)
    };
}

Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion) {
	// ベクトルをQuaternionに変換（w=0）
	Quaternion r{ vector.x, vector.y, vector.z, 0.0f };

	// クォータニオンの共役を計算
	Quaternion qConj{ -quaternion.x, -quaternion.y, -quaternion.z, quaternion.w };

	// q * r
	Quaternion qr;
	qr.x = quaternion.w * r.x + quaternion.x * r.w + quaternion.y * r.z - quaternion.z * r.y;
	qr.y = quaternion.w * r.y - quaternion.x * r.z + quaternion.y * r.w + quaternion.z * r.x;
	qr.z = quaternion.w * r.z + quaternion.x * r.y - quaternion.y * r.x + quaternion.z * r.w;
	qr.w = quaternion.w * r.w - quaternion.x * r.x - quaternion.y * r.y - quaternion.z * r.z;

	// (q * r) * qConj
	Quaternion resultQuat;
	resultQuat.x = qr.w * qConj.x + qr.x * qConj.w + qr.y * qConj.z - qr.z * qConj.y;
	resultQuat.y = qr.w * qConj.y - qr.x * qConj.z + qr.y * qConj.w + qr.z * qConj.x;
	resultQuat.z = qr.w * qConj.z + qr.x * qConj.y - qr.y * qConj.x + qr.z * qConj.w;
	resultQuat.w = qr.w * qConj.w - qr.x * qConj.x - qr.y * qConj.y - qr.z * qConj.z;

	// ベクトル部分を返す
	return Vector3{ resultQuat.x, resultQuat.y, resultQuat.z };
}

Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion) {
	Matrix4x4 m;

	float xx = quaternion.x * quaternion.x;
	float yy = quaternion.y * quaternion.y;
	float zz = quaternion.z * quaternion.z;
	float xy = quaternion.x * quaternion.y;
	float xz = quaternion.x * quaternion.z;
	float yz = quaternion.y * quaternion.z;
	float wx = quaternion.w * quaternion.x;
	float wy = quaternion.w * quaternion.y;
	float wz = quaternion.w * quaternion.z;

	m.m[0][0] = 1.0f - 2.0f * (yy + zz);
	m.m[0][1] = 2.0f * (xy + wz);
	m.m[0][2] = 2.0f * (xz - wy);
	m.m[0][3] = 0.0f;

	m.m[1][0] = 2.0f * (xy - wz);
	m.m[1][1] = 1.0f - 2.0f * (xx + zz);
	m.m[1][2] = 2.0f * (yz + wx);
	m.m[1][3] = 0.0f;

	m.m[2][0] = 2.0f * (xz + wy);
	m.m[2][1] = 2.0f * (yz - wx);
	m.m[2][2] = 1.0f - 2.0f * (xx + yy);
	m.m[2][3] = 0.0f;

	m.m[3][0] = 0.0f;
	m.m[3][1] = 0.0f;
	m.m[3][2] = 0.0f;
	m.m[3][3] = 1.0f;

	return m;
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

Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, float t) {
    // 内積を計算
    float dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;

    // q0 と q1 の内積が負の場合、q0 を反転させる
    Quaternion q1Adjusted = q1;
    if (dot < 0.0f) {
        q1Adjusted.x = -q1.x;
        q1Adjusted.y = -q1.y;
        q1Adjusted.z = -q1.z;
        q1Adjusted.w = -q1.w;
        dot = -dot;
    }

    // もし内積がほぼ1なら、線形補間を行う
    if (dot > 0.9995f) {
        return {
            q0.x + t * (q1Adjusted.x - q0.x),
            q0.y + t * (q1Adjusted.y - q0.y),
            q0.z + t * (q1Adjusted.z - q0.z),
            q0.w + t * (q1Adjusted.w - q0.w)
        };
    }

    // θ = acos(dot)
    float theta = std::acos(dot);
    float sinTheta = std::sin(theta);

    // スケールファクターの計算
    float scale0 = std::sin((1.0f - t) * theta) / sinTheta;
    float scale1 = std::sin(t * theta) / sinTheta;

    // 補間されたクォータニオンを計算
    return {
        scale0 * q0.x + scale1 * q1Adjusted.x,
        scale0 * q0.y + scale1 * q1Adjusted.y,
        scale0 * q0.z + scale1 * q1Adjusted.z,
        scale0 * q0.w + scale1 * q1Adjusted.w
    };
}
void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}

void QuaternionScreenPrintf(int x, int y, Quaternion quaternion, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + kColumnWidth * 4, y, label);
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

	Vector3 v1{ 1.0f, 3.0f, -5.0f };
	Vector3 v2{ 4.0f, -1.0f, 2.0f };

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


		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		Quaternion q0 = MakeRotateAxisAngleQuaternion({0.71f, 0.71f, 0.0f}, 0.3f);
		Quaternion q1 = MakeRotateAxisAngleQuaternion({ 0.71f, 0.0f, 0.71f }, 3.141592f);

		Quaternion interpolated0 = Slerp(q0, q1, 0.0f);
		Quaternion interpolated1 = Slerp(q0, q1, 0.3f);
		Quaternion interpolated2 = Slerp(q0, q1, 0.5f);
		Quaternion interpolated3 = Slerp(q0, q1, 0.7f);
		Quaternion interpolated4 = Slerp(q0, q1, 1.0f);

		QuaternionScreenPrintf(0, 20 * 0, interpolated0, "   : interpolated0");
		QuaternionScreenPrintf(0, 20 * 1, interpolated1, "   : interpolated1");
		QuaternionScreenPrintf(0, 20 * 2, interpolated2, "   : interpolated2");
		QuaternionScreenPrintf(0, 20 * 3, interpolated3, "   : interpolated3");
		QuaternionScreenPrintf(0, 20 * 4, interpolated4, "   : interpolated4");


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