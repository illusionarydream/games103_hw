using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	public bool launched = false;
	// * Baisc data
	Vector3[] X;
	Vector3[] Q;
	Vector3[] V;
	Matrix4x4 QQt = Matrix4x4.zero;
	Matrix4x4 Q_inv = Matrix4x4.zero;

	// * Physics parameters
	float gravity = 9.8f;
	float MIN_DIST = 0.04f;

	// * other objects
	public GameObject ground;
	public GameObject wall;


	// Start is called before the first frame update
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		V = new Vector3[mesh.vertices.Length]; // vertices velocity
		X = mesh.vertices; // vertices position
		Q = mesh.vertices; // vertices position in the local space

		//Centerizing Q.
		Vector3 c = Vector3.zero;
		for (int i = 0; i < Q.Length; i++)
			c += Q[i];
		c /= Q.Length;
		for (int i = 0; i < Q.Length; i++)
			Q[i] -= c;

		//Get QQ^t ready.
		for (int i = 0; i < Q.Length; i++)
		{
			QQt[0, 0] += Q[i][0] * Q[i][0];
			QQt[0, 1] += Q[i][0] * Q[i][1];
			QQt[0, 2] += Q[i][0] * Q[i][2];
			QQt[1, 0] += Q[i][1] * Q[i][0];
			QQt[1, 1] += Q[i][1] * Q[i][1];
			QQt[1, 2] += Q[i][1] * Q[i][2];
			QQt[2, 0] += Q[i][2] * Q[i][0];
			QQt[2, 1] += Q[i][2] * Q[i][1];
			QQt[2, 2] += Q[i][2] * Q[i][2];
		}
		QQt[3, 3] = 1;
		Q_inv = QQt.inverse;

		for (int i = 0; i < X.Length; i++)
			V[i][2] = 6.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position = Vector3.zero;
		transform.rotation = Quaternion.identity;
	}

	// Polar Decomposition that returns the rotation from F.
	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C[ii, jj] += F[kk, ii] * F[kk, jj];

		Matrix4x4 C2 = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C2[ii, jj] += C[ii, kk] * C[jj, kk];

		float det = F[0, 0] * F[1, 1] * F[2, 2] +
						F[0, 1] * F[1, 2] * F[2, 0] +
						F[1, 0] * F[2, 1] * F[0, 2] -
						F[0, 2] * F[1, 1] * F[2, 0] -
						F[0, 1] * F[1, 0] * F[2, 2] -
						F[0, 0] * F[1, 2] * F[2, 1];

		float I_c = C[0, 0] + C[1, 1] + C[2, 2];
		float I_c2 = I_c * I_c;
		float II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
		float III_c = det * det;
		float k = I_c2 - 3 * II_c;

		Matrix4x4 inv_U = Matrix4x4.zero;
		if (k < 1e-10f)
		{
			float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
			inv_U[0, 0] = inv_lambda;
			inv_U[1, 1] = inv_lambda;
			inv_U[2, 2] = inv_lambda;
		}
		else
		{
			float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
			float k_root = Mathf.Sqrt(k);
			float value = l / (k * k_root);
			if (value < -1.0f) value = -1.0f;
			if (value > 1.0f) value = 1.0f;
			float phi = Mathf.Acos(value);
			float lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
			float lambda = Mathf.Sqrt(lambda2);

			float III_u = Mathf.Sqrt(III_c);
			if (det < 0) III_u = -III_u;
			float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
			float II_u = (I_u * I_u - I_c) * 0.5f;


			float inv_rate, factor;
			inv_rate = 1 / (I_u * II_u - III_u);
			factor = I_u * III_u * inv_rate;

			Matrix4x4 U = Matrix4x4.zero;
			U[0, 0] = factor;
			U[1, 1] = factor;
			U[2, 2] = factor;

			factor = (I_u * I_u - II_u) * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

			inv_rate = 1 / III_u;
			factor = II_u * inv_rate;
			inv_U[0, 0] = factor;
			inv_U[1, 1] = factor;
			inv_U[2, 2] = factor;

			factor = -I_u * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
		}

		Matrix4x4 R = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					R[ii, jj] += F[ii, kk] * inv_U[kk, jj];
		R[3, 3] = 1;
		return R;
	}

	float SDF(Vector3 x, ref Vector3 P, ref Vector3 N)
	{
		//Get the signed distance function value of the plane

		Vector3 P2x = x - P;
		float d = Vector3.Dot(P2x, N);

		return d;
	}



	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
	{
		for (int i = 0; i < Q.Length; i++)
		{
			Vector3 x = (Vector3)(R * Q[i]) + c;

			V[i] += (x - X[i]) * inv_dt;
			X[i] = x;
		}

		// Mesh mesh = GetComponent<MeshFilter>().mesh;
		// mesh.vertices = X;
	}

	void Collision(float inv_dt, Vector3 P, Vector3 N)
	{

		// iterate through all the vertices
		for (int i = 0; i < X.Length; i++)
		{
			Vector3 x = X[i];
			Vector3 v = V[i];

			// * Collision with the ground
			float d = SDF(x, ref P, ref N);
			if (d < MIN_DIST)
			{
				V[i] = (MIN_DIST - d) * N * inv_dt;
				X[i] += (MIN_DIST - d) * N;
			}
		}
	}

	// Update is called once per frame
	void Update()
	{
		float dt = 0.015f;

		// * Step 1: run a simple particle system.
		for (int i = 0; i < V.Length; i++)
		{
			V[i][1] -= gravity * dt;
			X[i] += V[i] * dt;
		}

		// * Step 2: Perform simple particle collision.
		Collision(1 / dt, ground.transform.position, ground.transform.up);
		Collision(1 / dt, wall.transform.position, wall.transform.up);

		// * Step 3: Use shape matching to get new translation c and 
		// * new rotation R. Update the mesh by c and R.
		//Shape Matching (translation)
		Vector3 c = Vector3.zero;
		for (int i = 0; i < X.Length; i++)
			c += X[i];
		c /= X.Length;
		//Shape Matching (rotation)
		Matrix4x4 F = Matrix4x4.zero;
		for (int i = 0; i < X.Length; i++)
		{
			Matrix4x4 temp_F = Matrix4x4.zero;
			temp_F = MatrixExtensions.dot_T(X[i] - c, Q[i]);
			F = MatrixExtensions.Add(F, temp_F);
		}
		F = F * Q_inv;
		Matrix4x4 R = Get_Rotation(F);

		// * Step 4: Update the mesh by c and R.
		Update_Mesh(c, R, 1 / dt);

		transform.position = c;
		transform.rotation = R.rotation;
	}
}
