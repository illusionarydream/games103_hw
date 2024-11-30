using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using Unity.VisualScripting;

// * Math
public static class MatrixExtensions
{
	// Add two matrices
	public static Matrix4x4 Add(this Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 result = new Matrix4x4();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result[i, j] = a[i, j] + b[i, j];
			}
		}
		return result;
	}

	// subtract two matrices
	public static Matrix4x4 Subtract(this Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 result = new Matrix4x4();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result[i, j] = a[i, j] - b[i, j];
			}
		}
		return result;
	}

	// float multiply matrix
	public static Matrix4x4 Multiply(this Matrix4x4 a, float b)
	{
		Matrix4x4 result = new Matrix4x4();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result[i, j] = a[i, j] * b;
			}
		}
		return result;
	}

	// trace of matrix
	public static float Trace(this Matrix4x4 a)
	{
		float result = 0;
		for (int i = 0; i < 4; i++)
		{
			result += a[i, i];
		}
		return result;
	}

	// Add two quaternions
	public static Quaternion Add(this Quaternion a, Quaternion b)
	{
		Quaternion result = new Quaternion();
		result.x = a.x + b.x;
		result.y = a.y + b.y;
		result.z = a.z + b.z;
		result.w = a.w + b.w;
		return result;
	}

	// cross Vector3 and Vector3
	public static Vector3 Cross(this Vector3 a, Vector3 b)
	{
		Vector3 result = new Vector3();
		result.x = a.y * b.z - a.z * b.y;
		result.y = a.z * b.x - a.x * b.z;
		result.z = a.x * b.y - a.y * b.x;
		return result;
	}

	// dot Vector3 and Vector3^T
	public static Matrix4x4 dot_T(this Vector3 a, Vector3 b)
	{
		Matrix4x4 result = new Matrix4x4();
		result[0, 0] = a.x * b.x;
		result[0, 1] = a.x * b.y;
		result[0, 2] = a.x * b.z;
		result[1, 0] = a.y * b.x;
		result[1, 1] = a.y * b.y;
		result[1, 2] = a.y * b.z;
		result[2, 0] = a.z * b.x;
		result[2, 1] = a.z * b.y;
		result[2, 2] = a.z * b.z;
		return result;
	}
}

public class FVM : MonoBehaviour
{
	float dt = 0.003f;
	float mass = 1;
	float stiffness_0 = 20000.0f;
	float stiffness_1 = 5000.0f;
	float damp = 0.999f;
	float gravity = 9.8f;
	float uN = 0.5f;
	float uT = 0.5f;

	int[] Tet;
	int tet_number;         //The number of tetrahedra

	Vector3[] Force;
	Vector3[] V;
	Vector3[] X;
	int number;             //The number of vertices

	Matrix4x4[] inv_Dm;
	float[] det_inv_Dm_inv;

	//For Laplacian smoothing.
	float lambda = 0.1f;
	Vector3[] V_sum;
	int[] V_num;

	SVD svd = new SVD();

	// * for plane collision
	public GameObject plane;
	float MIN_surface = 0.01f;

	// Start is called before the first frame update
	void Start()
	{
		// FILO IO: Read the house model from files.
		// The model is from Jonathan Schewchuk's Stellar lib.
		// * read tetrahedra
		{
			string fileContent = File.ReadAllText("Assets/house2.ele");
			string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

			tet_number = int.Parse(Strings[0]);
			Tet = new int[tet_number * 4];

			for (int tet = 0; tet < tet_number; tet++)
			{
				Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
				Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
				Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
				Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
			}
		}
		// * read vertices
		{
			string fileContent = File.ReadAllText("Assets/house2.node");
			string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
			number = int.Parse(Strings[0]);
			X = new Vector3[number];
			for (int i = 0; i < number; i++)
			{
				X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
				X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
				X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
			}
			//Centralize the model.
			Vector3 center = Vector3.zero;
			for (int i = 0; i < number; i++) center += X[i];
			center = center / number;
			for (int i = 0; i < number; i++)
			{
				X[i] -= center;
				float temp = X[i].y;
				X[i].y = X[i].z;
				X[i].z = temp;
			}
		}

		//Create triangle mesh.
		Vector3[] vertices = new Vector3[tet_number * 12];
		int vertex_number = 0;
		for (int tet = 0; tet < tet_number; tet++)
		{
			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];

			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
		}

		int[] triangles = new int[tet_number * 12];
		for (int t = 0; t < tet_number * 4; t++)
		{
			triangles[t * 3 + 0] = t * 3 + 0;
			triangles[t * 3 + 1] = t * 3 + 1;
			triangles[t * 3 + 2] = t * 3 + 2;
		}
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals();

		V = new Vector3[number];
		Force = new Vector3[number];
		V_sum = new Vector3[number];
		V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		det_inv_Dm_inv = new float[tet_number];
		for (int tet = 0; tet < tet_number; tet++)
		{
			inv_Dm[tet] = Build_Edge_Matrix(tet);
			inv_Dm[tet] = inv_Dm[tet].inverse;
			// ! for more stable
			inv_Dm[tet].SetColumn(3, new Vector4(0, 0, 0, 1));
			inv_Dm[tet].SetRow(3, new Vector4(0, 0, 0, 1));

			det_inv_Dm_inv[tet] = 1 / inv_Dm[tet].determinant;
		}
	}

	Matrix4x4 Build_Edge_Matrix(int tet)
	{
		Matrix4x4 ret = Matrix4x4.zero;
		//TODO: Need to build edge matrix here.
		// set index
		int idx0 = Tet[tet * 4 + 0];
		int idx1 = Tet[tet * 4 + 1];
		int idx2 = Tet[tet * 4 + 2];
		int idx3 = Tet[tet * 4 + 3];
		// set X10, X20, X30
		Vector4 X10 = new Vector4(X[idx1].x - X[idx0].x, X[idx1].y - X[idx0].y, X[idx1].z - X[idx0].z, 0);
		Vector4 X20 = new Vector4(X[idx2].x - X[idx0].x, X[idx2].y - X[idx0].y, X[idx2].z - X[idx0].z, 0);
		Vector4 X30 = new Vector4(X[idx3].x - X[idx0].x, X[idx3].y - X[idx0].y, X[idx3].z - X[idx0].z, 0);
		// set Dm
		ret.SetColumn(0, X10);
		ret.SetColumn(1, X20);
		ret.SetColumn(2, X30);
		ret.SetColumn(3, new Vector4(0, 0, 0, 1));

		return ret;
	}


	void _Update()
	{
		// Jump up.
		if (Input.GetKeyDown(KeyCode.Space))
		{
			for (int i = 0; i < number; i++)
				V[i].y += 0.2f;
		}

		for (int i = 0; i < number; i++)
		{
			//TODO: Add gravity to Force.
			Force[i].x = 0;
			Force[i].y = -mass * gravity;
			Force[i].z = 0;
		}

		for (int tet = 0; tet < tet_number; tet++)
		{
			//TODO: Deformation Gradient
			Matrix4x4 F = Matrix4x4.zero;
			F = Build_Edge_Matrix(tet) * inv_Dm[tet];
			F.SetColumn(3, new Vector4(0, 0, 0, 1));

			//TODO: Green Strain
			Matrix4x4 G = F.transpose * F;
			G = MatrixExtensions.Subtract(G, Matrix4x4.identity);
			G = MatrixExtensions.Multiply(G, 0.5f);
			G.SetColumn(3, new Vector4(0, 0, 0, 0));


			//TODO: Second PK Stress
			float real_trace = G.Trace();
			Matrix4x4 S = MatrixExtensions.Add(
				MatrixExtensions.Multiply(Matrix4x4.identity, stiffness_0 * real_trace),
				MatrixExtensions.Multiply(G, 2 * stiffness_1)
			);
			S.SetColumn(3, new Vector4(0, 0, 0, 1));
			Matrix4x4 P = F * S;
			P.SetColumn(3, new Vector4(0, 0, 0, 1));


			//TODO: Elastic Force
			Matrix4x4 Forces = P * inv_Dm[tet].transpose;
			Forces = MatrixExtensions.Multiply(Forces, -det_inv_Dm_inv[tet] / 6);

			// parse the force
			Vector3 f1 = Forces.GetColumn(0);
			Vector3 f2 = Forces.GetColumn(1);
			Vector3 f3 = Forces.GetColumn(2);

			Force[Tet[tet * 4 + 1]] += f1;
			Force[Tet[tet * 4 + 2]] += f2;
			Force[Tet[tet * 4 + 3]] += f3;
			Force[Tet[tet * 4 + 0]] -= f1 + f2 + f3;
		}

		for (int i = 0; i < number; i++)
		{
			//TODO: Update X and V here.
			V[i] = V[i] * damp + Force[i] / mass * dt;
			X[i] += V[i] * dt;

			//TODO: (Particle) collision with floor.
			plane = GameObject.Find("Plane");
			Vector3 floorN = plane.transform.up;
			// assume the plane is horizontal
			if (X[i].y < plane.transform.position.y + MIN_surface)
			{
				X[i].y = plane.transform.position.y + MIN_surface;
				if (Vector3.Dot(V[i], floorN) < 0)
				{
					Vector3 vN = Vector3.Dot(V[i], floorN) * floorN;
					Vector3 vT = V[i] - vN;
					float a = Mathf.Max(1 - uT * (1 + uN) * vN.magnitude / vT.magnitude, 0);
					vN *= (-uN);
					vT *= a;
					V[i] = vN + vT;
				}
			}
		}

		// TODO: Laplacian smoothing
		for (int tet = 0; tet < tet_number; tet++)
		{
			int idx0 = Tet[tet * 4 + 0];
			int idx1 = Tet[tet * 4 + 1];
			int idx2 = Tet[tet * 4 + 2];
			int idx3 = Tet[tet * 4 + 3];
			V_sum[idx0] += V[idx1] + V[idx2] + V[idx3];
			V_num[idx0] += 3;
			V_sum[idx1] += V[idx0] + V[idx2] + V[idx3];
			V_num[idx1] += 3;
			V_sum[idx2] += V[idx0] + V[idx1] + V[idx3];
			V_num[idx2] += 3;
			V_sum[idx3] += V[idx0] + V[idx1] + V[idx2];
			V_num[idx3] += 3;
		}
		for (int i = 0; i < number; i++)
		{
			V[i] = (1 - lambda) * V[i] + lambda * V_sum[i] / V_num[i];
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;
		}
	}

	// Update is called once per frame
	void Update()
	{
		for (int l = 0; l < 10; l++)
			_Update();

		// Dump the vertex array for rendering.
		Vector3[] vertices = new Vector3[tet_number * 12];
		int vertex_number = 0;
		for (int tet = 0; tet < tet_number; tet++)
		{
			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
			vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
		}
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = vertices;
		mesh.RecalculateNormals();
	}
}
