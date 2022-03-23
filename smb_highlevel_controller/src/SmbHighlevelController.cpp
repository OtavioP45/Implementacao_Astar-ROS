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
		
		ROS_INFO_STREAM("O robo atingiu o checkpoint: " << numero_no); // DEBUG

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

	//ROS_INFO_STREAM(theta_obj << "	" << theta_atual); // DEBUG

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

	//ROS_INFO_STREAM("ERRO ANGULO" << erro_theta << "	" << msg_velocidade.angular.z << "	" << msg_velocidade.linear.x << "	" << numero_no); // DEBUG


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
