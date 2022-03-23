#include <smb_highlevel_controller/SmbHighlevelController.hpp>

#define PI 3.14159265359

namespace smb_highlevel_controller {

// Construtor
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{

	// Leitura dos parametros
	if(!LerParametros()){
		ROS_ERROR("Falha ao ler os parâmetros");
		ros::requestShutdown();
	}
	ROS_INFO("Sucesso ao ler os parâmetros");

	// Criacao do subscriber laser
	subscriber_laser_scan = nodeHandle_.subscribe(laser_topico, subscriber_tamanho_fila, &SmbHighlevelController::SubscriberCallback , this);

	// Criacao do subscriber posicao amcl
	subscriber_amcl = nodeHandle_.subscribe(amcl_topico, subscriber_tamanho_fila, &SmbHighlevelController::SubscriberAmclCallback , this);

	// Criacao do subscriber pontos caminho a*
	subscriber_caminho = nodeHandle_.subscribe(caminho_topico, subscriber_tamanho_fila, &SmbHighlevelController::SubscriberCaminhoCallback, this);

	// Criacao do publisher da velocidade
	publisher_velocidade_ = nodeHandle_.advertise<geometry_msgs::Twist>(velocidade_topico, subscriber_tamanho_fila);

	// Criacao do publisher do marcador do rviz
	marcador = nodeHandle_.advertise<visualization_msgs::Marker>( marcador_topico,  0);

	// Leitura dos pontos da trajetoria

	/*
	pontos_caminho.push_back({0.06835820895522306, 0.09856716417910434});
	pontos_caminho.push_back({0.5039701492537305, 0.09856716417910434});
	pontos_caminho.push_back({0.9395820895522379, 0.09856716417910434});
	pontos_caminho.push_back({1.3751940298507455, 0.09856716417910434});
	pontos_caminho.push_back({1.8108059701492527, -0.3370447761194031});
	pontos_caminho.push_back({2.2464179104477604, -0.7726567164179106});
	pontos_caminho.push_back({2.682029850746268, -1.208268656716418});
	pontos_caminho.push_back({3.1176417910447753, -1.208268656716418});
	pontos_caminho.push_back({3.5532537313432826, -1.208268656716418});
	pontos_caminho.push_back({3.9888656716417903, -1.208268656716418});
	pontos_caminho.push_back({4.424477611940298, -1.208268656716418});
	pontos_caminho.push_back({4.860089552238805, -1.208268656716418});
	pontos_caminho.push_back({5.295701492537313, -1.208268656716418});
	pontos_caminho.push_back({5.731313432835821, -1.208268656716418});
	pontos_caminho.push_back({6.166925373134328, -1.6438805970149255});
	pontos_caminho.push_back({6.602537313432835, -2.079492537313433});
	pontos_caminho.push_back({7.038149253731342, -2.5151044776119402});
	pontos_caminho.push_back({7.038149253731342, -2.950716417910448});
	pontos_caminho.push_back({7.038149253731342, -3.3863283582089556});
	pontos_caminho.push_back({6.602537313432835, -3.821940298507463});
	pontos_caminho.push_back({6.166925373134328, -4.25755223880597});
	pontos_caminho.push_back({5.731313432835821, -4.25755223880597});
	pontos_caminho.push_back({5.295701492537313, -4.25755223880597});
	pontos_caminho.push_back({4.860089552238805, -4.25755223880597});
	pontos_caminho.push_back({4.424477611940298, -4.25755223880597});
	pontos_caminho.push_back({3.9888656716417903, -4.25755223880597});
	pontos_caminho.push_back({3.5532537313432826, -4.25755223880597});
	pontos_caminho.push_back({3.1176417910447753, -4.25755223880597});
	pontos_caminho.push_back({2.682029850746268, -4.25755223880597});
	pontos_caminho.push_back({2.2464179104477604, -4.25755223880597});
	pontos_caminho.push_back({1.8108059701492527, -4.25755223880597});
	pontos_caminho.push_back({1.3751940298507455, -4.25755223880597});
	pontos_caminho.push_back({0.9395820895522379, -4.25755223880597});
	pontos_caminho.push_back({0.5039701492537305, -4.25755223880597});
	pontos_caminho.push_back({0.06835820895522306, -4.25755223880597});
	pontos_caminho.push_back({-0.36725373134328443, -4.25755223880597});
	pontos_caminho.push_back({-0.8028656716417919, -4.25755223880597});
	pontos_caminho.push_back({-1.2384776119402994, -4.25755223880597});
	pontos_caminho.push_back({-1.6740895522388068, -4.25755223880597});
	pontos_caminho.push_back({-2.1097014925373143, -4.25755223880597});
	pontos_caminho.push_back({-2.5453134328358216, -4.25755223880597});
	pontos_caminho.push_back({-2.9809253731343293, -4.25755223880597});
	pontos_caminho.push_back({-3.4165373134328365, -4.25755223880597});
	pontos_caminho.push_back({-3.852149253731344, -3.821940298507463});
	pontos_caminho.push_back({-4.2877611940298515, -3.3863283582089556});
	pontos_caminho.push_back({-4.2877611940298515, -2.950716417910448});
	pontos_caminho.push_back({-4.2877611940298515, -2.5151044776119402});
	pontos_caminho.push_back({-4.2877611940298515, -2.079492537313433});
	pontos_caminho.push_back({-4.2877611940298515, -1.6438805970149255});
	pontos_caminho.push_back({-4.2877611940298515, -1.208268656716418});
	pontos_caminho.push_back({-4.2877611940298515, -0.7726567164179106});
	pontos_caminho.push_back({-4.2877611940298515, -0.3370447761194031});
	pontos_caminho.push_back({-4.2877611940298515, 0.09856716417910434});
	pontos_caminho.push_back({-4.2877611940298515, 0.5341791044776119});
	pontos_caminho.push_back({-4.2877611940298515, 0.9697910447761192});
	pontos_caminho.push_back({-4.2877611940298515, 1.4054029850746268});
	pontos_caminho.push_back({-4.2877611940298515, 1.8410149253731343});
	pontos_caminho.push_back({-3.852149253731344, 2.2766268656716417});
	pontos_caminho.push_back({-3.4165373134328365, 2.712238805970149});
	pontos_caminho.push_back({-2.9809253731343293, 2.712238805970149});
	pontos_caminho.push_back({-2.5453134328358216, 2.712238805970149});
	pontos_caminho.push_back({-2.1097014925373143, 3.1478507462686567});
	pontos_caminho.push_back({-1.6740895522388068, 3.583462686567164});
	pontos_caminho.push_back({-1.2384776119402994, 4.019074626865671});
	pontos_caminho.push_back({-0.8028656716417919, 4.454686567164178});
	pontos_caminho.push_back({-0.36725373134328443, 4.890298507462687});
	pontos_caminho.push_back({0.06835820895522306, 5.325910447761194});
	pontos_caminho.push_back({0.5039701492537305, 5.325910447761194});
	pontos_caminho.push_back({0.9395820895522379, 5.325910447761194});
	pontos_caminho.push_back({1.3751940298507455, 5.325910447761194});
	pontos_caminho.push_back({1.8108059701492527, 5.325910447761194});
	pontos_caminho.push_back({2.2464179104477604, 5.325910447761194});
	pontos_caminho.push_back({2.682029850746268, 5.325910447761194});
	pontos_caminho.push_back({3.1176417910447753, 5.325910447761194});
	pontos_caminho.push_back({3.5532537313432826, 5.325910447761194});
	pontos_caminho.push_back({3.9888656716417903, 5.325910447761194});
	pontos_caminho.push_back({4.424477611940298, 5.325910447761194});
	pontos_caminho.push_back({4.860089552238805, 5.325910447761194});
	pontos_caminho.push_back({5.295701492537313, 5.325910447761194});
	pontos_caminho.push_back({5.731313432835821, 5.325910447761194});
	pontos_caminho.push_back({6.166925373134328, 5.325910447761194});
	pontos_caminho.push_back({6.602537313432835, 5.325910447761194});
	pontos_caminho.push_back({7.038149253731342, 5.325910447761194});
	pontos_caminho.push_back({7.4737611940298505, 5.761522388059702});
	pontos_caminho.push_back({7.909373134328358, 6.197134328358209});
	pontos_caminho.push_back({7.909373134328358, 6.6327462686567165});
	pontos_caminho.push_back({7.909373134328358, 7.068358208955224});
	pontos_caminho.push_back({7.909373134328358, 7.503970149253731});
	pontos_caminho.push_back({7.909373134328358, 7.939582089552239});
	pontos_caminho.push_back({7.909373134328358, 8.375194029850746});
	pontos_caminho.push_back({7.909373134328358, 8.810805970149254});
	pontos_caminho.push_back({7.909373134328358, 9.24641791044776});
	pontos_caminho.push_back({7.909373134328358, 9.68202985074627});
	pontos_caminho.push_back({7.909373134328358, 10.117641791044777});
	pontos_caminho.push_back({7.909373134328358, 10.553253731343284});
	pontos_caminho.push_back({7.909373134328358, 10.988865671641792});
	pontos_caminho.push_back({7.909373134328358, 11.424477611940299});
	pontos_caminho.push_back({7.909373134328358, 11.860089552238806});
	pontos_caminho.push_back({7.909373134328358, 12.295701492537313});
	pontos_caminho.push_back({7.909373134328358, 12.731313432835822});
	pontos_caminho.push_back({7.909373134328358, 13.16692537313433});
	pontos_caminho.push_back({7.909373134328358, 13.602537313432837});
	pontos_caminho.push_back({7.909373134328358, 14.038149253731344});
	pontos_caminho.push_back({7.909373134328358, 14.473761194029851});
	pontos_caminho.push_back({7.909373134328358, 14.909373134328359});
	pontos_caminho.push_back({7.909373134328358, 15.344985074626866});
	pontos_caminho.push_back({7.909373134328358, 15.780597014925373});
	pontos_caminho.push_back({7.909373134328358, 16.21620895522388});
	pontos_caminho.push_back({7.4737611940298505, 16.651820895522388});
	pontos_caminho.push_back({7.038149253731342, 17.087432835820895});
	pontos_caminho.push_back({6.602537313432835, 17.087432835820895});
	pontos_caminho.push_back({6.166925373134328, 17.087432835820895});
	pontos_caminho.push_back({5.731313432835821, 16.651820895522388});
	pontos_caminho.push_back({5.295701492537313, 16.21620895522388});
	pontos_caminho.push_back({4.860089552238805, 15.780597014925373});
	pontos_caminho.push_back({4.424477611940298, 15.344985074626866});
	pontos_caminho.push_back({3.9888656716417903, 14.909373134328359});
	pontos_caminho.push_back({3.5532537313432826, 14.473761194029851});
	pontos_caminho.push_back({3.1176417910447753, 14.038149253731344});
	pontos_caminho.push_back({3.1176417910447753, 13.602537313432837});
	pontos_caminho.push_back({3.1176417910447753, 13.16692537313433});
	pontos_caminho.push_back({2.682029850746268, 12.731313432835822});
	pontos_caminho.push_back({2.2464179104477604, 12.295701492537313});
	pontos_caminho.push_back({1.8108059701492527, 12.295701492537313});
	pontos_caminho.push_back({1.3751940298507455, 12.295701492537313});
	pontos_caminho.push_back({0.9395820895522379, 12.295701492537313});
	pontos_caminho.push_back({0.5039701492537305, 12.295701492537313});
	pontos_caminho.push_back({0.06835820895522306, 12.295701492537313});
	pontos_caminho.push_back({-0.36725373134328443, 12.295701492537313});
	pontos_caminho.push_back({-0.8028656716417919, 12.295701492537313});
	pontos_caminho.push_back({-1.2384776119402994, 12.295701492537313});
	pontos_caminho.push_back({-1.6740895522388068, 12.295701492537313});
	pontos_caminho.push_back({-2.1097014925373143, 12.295701492537313});
	pontos_caminho.push_back({-2.5453134328358216, 12.295701492537313});
	pontos_caminho.push_back({-2.9809253731343293, 12.295701492537313});
	pontos_caminho.push_back({-3.4165373134328365, 12.295701492537313});
	pontos_caminho.push_back({-3.852149253731344, 12.731313432835822});
	pontos_caminho.push_back({-4.2877611940298515, 13.16692537313433});
	pontos_caminho.push_back({-4.2877611940298515, 13.602537313432837});
	pontos_caminho.push_back({-4.723373134328359, 14.038149253731344});
	pontos_caminho.push_back({-4.723373134328359, 14.473761194029851});
	pontos_caminho.push_back({-5.158985074626866, 14.909373134328359});
	pontos_caminho.push_back({-5.158985074626866, 15.344985074626866});
	pontos_caminho.push_back({-5.594597014925374, 15.780597014925373});
	pontos_caminho.push_back({-5.594597014925374, 16.21620895522388});
	pontos_caminho.push_back({-5.594597014925374, 16.651820895522388});
	pontos_caminho.push_back({-6.030208955223881, 17.087432835820895});
	*/

	/*
	pontos_caminho.push_back({0.0, 0.0});
	pontos_caminho.push_back({0.5, 0.0});
	pontos_caminho.push_back({1.0, 0.0});
	pontos_caminho.push_back({1.5, 0.0});
	pontos_caminho.push_back({2.0, 0.0});
	pontos_caminho.push_back({2.5, 0.0});
	pontos_caminho.push_back({3.0, 0.0});
	pontos_caminho.push_back({3.5, -0.5});
	pontos_caminho.push_back({4.0, -1.0});
	pontos_caminho.push_back({4.5, -1.0});
	pontos_caminho.push_back({5.0, -1.0});
	pontos_caminho.push_back({5.5, -1.0});
	pontos_caminho.push_back({6.0, -1.0});
	pontos_caminho.push_back({6.5, -1.0});
	pontos_caminho.push_back({7.0, -1.0});
	pontos_caminho.push_back({7.5, -1.0});
	pontos_caminho.push_back({8.0, -1.5});
	pontos_caminho.push_back({8.0, -2.0});
	pontos_caminho.push_back({8.0, -2.5});
	pontos_caminho.push_back({7.5, -3.0});
	pontos_caminho.push_back({7.0, -3.5});
	pontos_caminho.push_back({6.5, -3.5});
	pontos_caminho.push_back({6.0, -3.5});
	pontos_caminho.push_back({5.5, -3.5});
	pontos_caminho.push_back({5.0, -3.5});
	pontos_caminho.push_back({4.5, -3.5});
	pontos_caminho.push_back({4.0, -3.5});
	pontos_caminho.push_back({3.5, -3.5});
	pontos_caminho.push_back({3.0, -3.5});
	pontos_caminho.push_back({2.5, -3.5});
	pontos_caminho.push_back({2.0, -3.5});
	pontos_caminho.push_back({1.5, -3.5});
	pontos_caminho.push_back({1.0, -3.5});
	pontos_caminho.push_back({0.5, -3.5});
	pontos_caminho.push_back({0.0, -3.5});
	pontos_caminho.push_back({-0.5, -3.5});
	pontos_caminho.push_back({-1.0, -3.5});
	pontos_caminho.push_back({-1.5, -3.0});
	pontos_caminho.push_back({-2.0, -3.0});
	pontos_caminho.push_back({-2.5, -2.5});
	pontos_caminho.push_back({-2.5, -2.0});
	pontos_caminho.push_back({-2.5, -1.5});
	pontos_caminho.push_back({-2.5, -1.0});
	pontos_caminho.push_back({-2.5, -0.5});
	pontos_caminho.push_back({-2.5, 0.0});
	pontos_caminho.push_back({-2.5, 0.5});
	pontos_caminho.push_back({-2.5, 1.0});
	pontos_caminho.push_back({-2.5, 1.5});
	pontos_caminho.push_back({-2.5, 2.0});
	pontos_caminho.push_back({-2.0, 2.5});
	pontos_caminho.push_back({-1.5, 3.0});
	pontos_caminho.push_back({-1.0, 3.5});
	pontos_caminho.push_back({-0.5, 4.0});
	pontos_caminho.push_back({0.0, 4.5});
	pontos_caminho.push_back({0.5, 5.0});
	pontos_caminho.push_back({1.0, 5.5});
	pontos_caminho.push_back({1.5, 6.0});
	pontos_caminho.push_back({2.0, 6.0});
	pontos_caminho.push_back({2.5, 6.0});
	pontos_caminho.push_back({3.0, 6.0});
	pontos_caminho.push_back({3.5, 6.0});
	pontos_caminho.push_back({4.0, 6.0});
	pontos_caminho.push_back({4.5, 6.0});
	pontos_caminho.push_back({5.0, 6.0});
	pontos_caminho.push_back({5.5, 6.0});
	pontos_caminho.push_back({6.0, 6.0});
	pontos_caminho.push_back({6.5, 6.0});
	pontos_caminho.push_back({7.0, 6.0});
	pontos_caminho.push_back({7.5, 6.0});
	pontos_caminho.push_back({8.0, 6.5});
	pontos_caminho.push_back({8.0, 7.0});
	pontos_caminho.push_back({8.0, 7.5});
	pontos_caminho.push_back({8.0, 8.0});
	pontos_caminho.push_back({8.0, 8.5});
	pontos_caminho.push_back({8.0, 9.0});
	pontos_caminho.push_back({8.0, 9.5});
	pontos_caminho.push_back({8.0, 10.0});
	pontos_caminho.push_back({8.0, 10.5});
	pontos_caminho.push_back({8.0, 11.0});
	pontos_caminho.push_back({8.0, 11.5});
	pontos_caminho.push_back({8.0, 12.0});
	pontos_caminho.push_back({8.0, 12.5});
	pontos_caminho.push_back({8.0, 13.0});
	pontos_caminho.push_back({8.0, 13.5});
	pontos_caminho.push_back({8.0, 14.0});
	pontos_caminho.push_back({8.0, 14.5});
	pontos_caminho.push_back({8.0, 15.0});
	pontos_caminho.push_back({8.0, 15.5});
	pontos_caminho.push_back({8.0, 16.0});
	pontos_caminho.push_back({8.0, 16.5});
	pontos_caminho.push_back({8.0, 17.0});
	pontos_caminho.push_back({7.5, 17.5});
	pontos_caminho.push_back({7.0, 17.5});
	pontos_caminho.push_back({6.5, 17.0});
	pontos_caminho.push_back({6.0, 16.5});
	pontos_caminho.push_back({5.5, 16.0});
	pontos_caminho.push_back({5.0, 15.5});
	pontos_caminho.push_back({4.5, 15.0});
	pontos_caminho.push_back({4.5, 14.5});
	pontos_caminho.push_back({4.5, 14.0});
	pontos_caminho.push_back({4.5, 13.5});
	pontos_caminho.push_back({4.0, 13.0});
	pontos_caminho.push_back({3.5, 12.5});
	pontos_caminho.push_back({3.0, 12.5});
	pontos_caminho.push_back({2.5, 12.5});
	pontos_caminho.push_back({2.0, 12.5});
	pontos_caminho.push_back({1.5, 12.5});
	pontos_caminho.push_back({1.0, 12.5});
	pontos_caminho.push_back({0.5, 12.5});
	pontos_caminho.push_back({0.0, 12.5});
	pontos_caminho.push_back({-0.5, 12.5});
	pontos_caminho.push_back({-1.0, 12.5});
	pontos_caminho.push_back({-1.5, 13.0});
	pontos_caminho.push_back({-2.0, 13.0});
	pontos_caminho.push_back({-2.5, 13.5});
	pontos_caminho.push_back({-3.0, 14.0});
	pontos_caminho.push_back({-3.5, 14.5});
	pontos_caminho.push_back({-3.5, 15.0});
	pontos_caminho.push_back({-3.5, 15.5});
	pontos_caminho.push_back({-3.5, 16.0});
	pontos_caminho.push_back({-3.5, 16.5});
	pontos_caminho.push_back({-3.5, 17.0});
	pontos_caminho.push_back({-3.5, 17.5});
	pontos_caminho.push_back({-4.0, 18.0});
	*/

	// Primeiro ponto do caminho
	float x_obj = pontos_caminho[0][0];
	float y_obj = pontos_caminho[0][1];

}

// Metodo para ler parametros
bool SmbHighlevelController::LerParametros(){

	if(!nodeHandle_.getParam("topico_laser", laser_topico)) return false; // Get nome do topico laser
	if(!nodeHandle_.getParam("topico_amcl", amcl_topico)) return false; // Get nome do topico amcl
	if(!nodeHandle_.getParam("topico_caminho_a_estrela", caminho_topico)) return false; // Get nome do topico caminho a*
	if(!nodeHandle_.getParam("topico_velocidade", velocidade_topico)) return false; // Get nome do topico velocidade
	if(!nodeHandle_.getParam("topico_marcador", marcador_topico)) return false; // Get nome do topico marcador
	if(!nodeHandle_.getParam("tamanho_fila_subscriber", subscriber_tamanho_fila)) return false; // Get tamanho fila
	if(!nodeHandle_.getParam("ganho_linear", K_LINEAR_L)) return false; // Get ganho controlador velocidade linear
	if(!nodeHandle_.getParam("ganho_angular", K_ANGULAR_L)) return false; // Get ganho controlador velocidade angular
	if(!nodeHandle_.getParam("tamanho_objetivo", INTERVALO_OBJ)) return false; // Get tamanho do ponto do objetivo
	if(!nodeHandle_.getParam("alcance_controlador_local", ALCANCE_LOCAL)) return false; // Get tamanho do ponto do objetivo
	if(!nodeHandle_.getParam("velocidade_angular_maxima", V_ANGULAR_MAX)) return false; // Get velocidade angular maxima

	return true;

}

void SmbHighlevelController::SubscriberCaminhoCallback(const geometry_msgs::PoseArray msg){

	// Marcador trajetoria
	visualization_msgs::Marker trajetoria;
	trajetoria.header.frame_id = "map";
	trajetoria.header.stamp = ros::Time::now();
	trajetoria.id = 1;
	trajetoria.type = visualization_msgs::Marker::LINE_STRIP;
	trajetoria.action = visualization_msgs::Marker::ADD;
	trajetoria.scale.x = 0.1;
	trajetoria.scale.y = 0.1;
	trajetoria.scale.z = 0.1;
	trajetoria.color.a = 0.6;
	trajetoria.color.r = 1.0;
	trajetoria.color.g = 0.0;
	trajetoria.color.b = 1.0;

	// Leitura da trajetoria gerada pelo planejador global a*
	for(int i=0; i<msg.poses.size(); i++){
		pontos_caminho.push_back({(float)msg.poses[i].position.x, (float)msg.poses[i].position.y});
		trajetoria.points.push_back(msg.poses[i].position);
	}

	// Publica marcador
	marcador.publish(trajetoria);

	// Desliga o subscriber do caminho
	subscriber_caminho.shutdown();

	// Set flag caminho recebido
	caminho_recebido = true;

	return;
}

// Callback subscriber posicao amcl
void SmbHighlevelController::SubscriberAmclCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

	if(!caminho_recebido) return; // Retorna a funcao se o caminho ainda nao foi recebido

	// Posicao atual do robo
	x_atual = msg.pose.pose.position.x;
	y_atual = msg.pose.pose.position.y;

	// Conversao da orientacao para angulo
	theta_atual = atan2( (2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)),  (msg.pose.pose.orientation.w * msg.pose.pose.orientation.w + msg.pose.pose.orientation.x * msg.pose.pose.orientation.x - msg.pose.pose.orientation.y * msg.pose.pose.orientation.y - msg.pose.pose.orientation.z * msg.pose.pose.orientation.z ));

	// Verifica se o robo chegou ao objetivo
	if(abs(x_atual - x_obj) < INTERVALO_OBJ && abs(y_atual - y_obj) < INTERVALO_OBJ){

		// Atualizacao do objetivo
		if(numero_no+1 < pontos_caminho.size()) numero_no++;
		else if (parar_robo == false) {
			parar_robo = true;
			ROS_INFO_STREAM("O robo atingiu seu objetivo"); // DEBUG
		}

		x_obj = pontos_caminho[numero_no][0];
		y_obj = pontos_caminho[numero_no][1];

		// Marcador para o novo objetivo
		visualization_msgs::Marker ponto;
		ponto.header.frame_id = "map";
		ponto.header.stamp = ros::Time::now();
		ponto.id = 0;
		ponto.type = visualization_msgs::Marker::SPHERE;
		ponto.action = visualization_msgs::Marker::ADD;
		ponto.pose.position.x = x_obj;
		ponto.pose.position.y = y_obj;
		ponto.pose.position.z = 0.0;
		ponto.scale.x = INTERVALO_OBJ;
		ponto.scale.y = INTERVALO_OBJ;
		ponto.scale.z = INTERVALO_OBJ;
		ponto.color.a = 0.6;
		ponto.color.r = 0.0;
		ponto.color.g = 1.0;
		ponto.color.b = 1.0;

		// Publica marcador
		marcador.publish(ponto);

	}

	// Marcador para o alcance do controlador local
	visualization_msgs::Marker ctrl_local;
	ctrl_local.header.frame_id = "map";
	ctrl_local.header.stamp = ros::Time::now();
	ctrl_local.id = 2;
	ctrl_local.type = visualization_msgs::Marker::CYLINDER;
	ctrl_local.action = visualization_msgs::Marker::ADD;
	ctrl_local.pose.position.x = x_atual;
	ctrl_local.pose.position.y = y_atual;
	ctrl_local.pose.position.z = 0.0;
	ctrl_local.pose.orientation.w = 1.0;
	ctrl_local.scale.x = 2*ALCANCE_LOCAL;
	ctrl_local.scale.y = 2*ALCANCE_LOCAL;
	ctrl_local.scale.z = 0.1;
	ctrl_local.color.a = 0.1;
	ctrl_local.color.r = 1.0;
	ctrl_local.color.g = 0.0;
	ctrl_local.color.b = 0.0;

	// Publica marcador
	marcador.publish(ctrl_local);

}

// Callback subscriber laser scan
void SmbHighlevelController::SubscriberCallback(const sensor_msgs::LaserScan& msg){

	if(!caminho_recebido) return; // Retorna a funcao se o caminho ainda nao foi recebido

	// Para o robo caso ele tenha chegado ao fim da trajetoria
	if (parar_robo == true){
		msg_velocidade.angular.z = 0;
		msg_velocidade.linear.x = 0;
		publisher_velocidade_.publish(msg_velocidade);
		return;
	}

	// Calcula o angulo do objetivo
	theta_obj = atan( (y_obj - y_atual) / (x_obj - x_atual) );
	if(x_obj  < x_atual) theta_obj -= PI;

	ROS_INFO_STREAM(theta_obj << "	" << theta_atual); // DEBUG

	// Calcula o erro do angulo
	float erro_theta = theta_obj - theta_atual;

	if(erro_theta > PI) erro_theta = erro_theta - 2*PI;
	if(erro_theta < -PI) erro_theta = erro_theta + 2* PI;

	if (erro_theta > 0 || erro_theta < -0){
		msg_velocidade.angular.z =  0.6 * erro_theta;

		if (msg_velocidade.angular.z > V_ANGULAR_MAX) msg_velocidade.angular.z = V_ANGULAR_MAX; // Limita a velocidade angular
		if (msg_velocidade.angular.z < -V_ANGULAR_MAX) msg_velocidade.angular.z = -V_ANGULAR_MAX; // Limita a velocidade angular

		msg_velocidade.linear.x = 0.4 / (0.4 + K_LINEAR_L * abs(erro_theta) );
	}
	else{
		msg_velocidade.angular.z = 0;
		msg_velocidade.linear.x = 0.5;
	}

	ROS_INFO_STREAM("ERRO ANGULO" << erro_theta << "	" << msg_velocidade.angular.z << "	" << msg_velocidade.linear.x << "	" << numero_no); // DEBUG


	// Controlador Local
	auto minimo1 = msg.range_max;
	float erro_angulo1 = 0;
	bool n_min = false;

	for(int i =0; i<msg.ranges.size(); i++){
		if (msg.ranges[i] > msg.range_min && msg.ranges[i] < ALCANCE_LOCAL){

			if( msg.ranges[i] < minimo1 && (msg.angle_min + i * msg.angle_increment) < (0.7*PI/2.0) && (msg.angle_min + i * msg.angle_increment) > -(0.7*PI/2.0)){
				minimo1 = msg.ranges[i];
				erro_angulo1 = msg.angle_min + i * msg.angle_increment;
				n_min = true;
			}

		}
	}


	if(	n_min == true	){
		if( erro_angulo1 < 0) msg_velocidade.angular.z = K_ANGULAR_L * (1 / (5 + minimo1) ) * (cos(erro_angulo1));
		else msg_velocidade.angular.z = -K_ANGULAR_L * (1 / (5 + minimo1) ) * (cos(erro_angulo1));
	}

	publisher_velocidade_.publish(msg_velocidade);
}

// Destrutor
SmbHighlevelController::~SmbHighlevelController()
{
}

} /* namespace */
