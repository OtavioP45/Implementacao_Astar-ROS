#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <nav_msgs/Odometry.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:

	// Atributos
	ros::NodeHandle nodeHandle_;

	ros::Subscriber subscriber_laser_scan; // Subscriber laser
	ros::Subscriber subscriber_amcl; // Subscriber posicao amcl
	ros::Subscriber subscriber_caminho; // Subscriber pontos do caminho gerado pelo a*

	ros::Publisher publisher_velocidade_; // Publisher velocidade
	ros::Publisher marcador; // Publisher marcador rviz

	geometry_msgs::Twist msg_velocidade; 	// Mensagem velocidade

	std::vector<std::array<float, 2>> pontos_caminho; // Vetor de pontos do caminho

	int numero_no = 0; // Ponto atual do caminho

	float x_atual = 0; // Posicao x atual do robo
	float y_atual = 0; // Posicao y atual do robo
	float theta_atual = 0; // Angulo atual do robo

	bool parar_robo = false; // parada do movimento do robo
	bool caminho_recebido = false;

	float x_obj = 0; // Posicao x do objetivo
	float y_obj = 0; // Posicao y do objetivo
	float theta_obj = 0; // Angulo do objetivo

	// Metodos =========================================
	bool LerParametros(); // Metodo para ler parametros
	void SubscriberCallback(const sensor_msgs::LaserScan& msg); // Callback subscriber laser
	void SubscriberAmclCallback(const geometry_msgs::PoseWithCovarianceStamped msg); // Callback subscriber posicao amcl
	void SubscriberCaminhoCallback(const geometry_msgs::PoseArray msg); // Callback subscriber caminho a*

	// Parametros =================================
	float K_LINEAR_L; // Ganho velocidade linear do controlador local
	float K_ANGULAR_L; // Ganho velocidade angular do controlador local
	int subscriber_tamanho_fila; // Tamanho fila subscriber laser
	float INTERVALO_OBJ; // Tamanho do ponto de objetivo
	float ALCANCE_LOCAL; // Alcance de atuacao do controlador local
	float V_ANGULAR_MAX; // Velocidade angular maxima

	std::string laser_topico; // Nome topico subscriber laser
	std::string amcl_topico; // Nome topico subscriber amcl
	std::string velocidade_topico; // Nome topico comando velocidade
	std::string marcador_topico; // Nome topico marcador rviz
	std::string caminho_topico; // Nome topico caminho a*


};

} /* namespace */
