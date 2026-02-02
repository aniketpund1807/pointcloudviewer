-- phpMyAdmin SQL Dump
-- version 5.2.1
-- https://www.phpmyadmin.net/
--
-- Host: 127.0.0.1
-- Generation Time: Jan 28, 2026 at 07:02 AM
-- Server version: 10.4.32-MariaDB
-- PHP Version: 8.0.30

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `integrated`
--

-- --------------------------------------------------------

--
-- Table structure for table `authority_header_all`
--

CREATE TABLE `authority_header_all` (
  `auth_id` varchar(25) NOT NULL,
  `auth_name` varchar(50) NOT NULL,
  `auth_description` text NOT NULL,
  `auth_form_id` longtext NOT NULL COMMENT 'csv',
  `auth_process_id` longtext NOT NULL COMMENT 'csv',
  `auth_inserted_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `bridge_layer_data_details_all`
--

CREATE TABLE `bridge_layer_data_details_all` (
  `lbad_id` int(255) NOT NULL,
  `worksheet_id` varchar(20) DEFAULT NULL COMMENT 'project_worksheet_header_all',
  `layer_id` varchar(20) DEFAULT NULL COMMENT 'worksheet_layer_header_all',
  `bridge_type` tinyint(1) DEFAULT NULL COMMENT '1=Underpass(single), 2=Underpass(double), 3=Bridge, 4=Pipe-based crossover',
  `from_km` int(11) DEFAULT NULL,
  `from_chainage` int(11) DEFAULT NULL,
  `to_km` int(11) DEFAULT NULL,
  `to_chainage` int(11) DEFAULT NULL,
  `line_no` int(11) DEFAULT NULL,
  `cp1_id` varchar(20) DEFAULT NULL,
  `cp1_foundation_data` longtext DEFAULT NULL COMMENT 'JSON format',
  `cp1_base_data` longtext DEFAULT NULL COMMENT 'JSON format',
  `cp1_pillar_data` longtext DEFAULT NULL COMMENT 'JSON format',
  `cp1_span_data` longtext DEFAULT NULL COMMENT 'JSON format',
  `cp2_id` varchar(20) DEFAULT NULL,
  `cp2_foundation_data` longtext DEFAULT NULL,
  `cp2_base_data` longtext DEFAULT NULL,
  `cp2_pillar_data` longtext DEFAULT NULL,
  `cp2_span_data` longtext DEFAULT NULL,
  `cp3_id` varchar(20) DEFAULT NULL,
  `cp3_foundation_data` longtext DEFAULT NULL,
  `cp3_base_data` longtext DEFAULT NULL,
  `cp3_pillar_data` longtext DEFAULT NULL,
  `cp3_span_data` longtext DEFAULT NULL,
  `cp4_id` varchar(20) DEFAULT NULL,
  `cp4_foundation_data` longtext DEFAULT NULL,
  `cp4_base_data` longtext DEFAULT NULL,
  `cp4_pillar_data` longtext DEFAULT NULL,
  `cp4_span_data` longtext DEFAULT NULL,
  `cp5_id` varchar(20) DEFAULT NULL,
  `cp5_foundation_data` longtext DEFAULT NULL,
  `cp5_base_data` longtext DEFAULT NULL,
  `cp5_pillar_data` longtext DEFAULT NULL,
  `cp5_span_data` longtext DEFAULT NULL,
  `cp6_id` varchar(20) DEFAULT NULL,
  `cp6_foundation_data` longtext DEFAULT NULL,
  `cp6_base_data` longtext DEFAULT NULL,
  `cp6_pillar_data` longtext DEFAULT NULL,
  `cp6_span_data` longtext DEFAULT NULL,
  `cp7_id` varchar(20) DEFAULT NULL,
  `cp7_foundation_data` longtext DEFAULT NULL,
  `cp7_base_data` longtext DEFAULT NULL,
  `cp7_pillar_data` longtext DEFAULT NULL,
  `cp7_span_data` longtext DEFAULT NULL,
  `cp8_id` varchar(20) DEFAULT NULL,
  `cp8_foundation_data` longtext DEFAULT NULL,
  `cp8_base_data` longtext DEFAULT NULL,
  `cp8_pillar_data` longtext DEFAULT NULL,
  `cp8_span_data` longtext DEFAULT NULL,
  `cp9_id` varchar(20) DEFAULT NULL,
  `cp9_foundation_data` longtext DEFAULT NULL,
  `cp9_base_data` longtext DEFAULT NULL,
  `cp9_pillar_data` longtext DEFAULT NULL,
  `cp9_span_data` longtext DEFAULT NULL,
  `cp10_id` varchar(20) DEFAULT NULL,
  `cp10_foundation_data` longtext DEFAULT NULL,
  `cp10_base_data` longtext DEFAULT NULL,
  `cp10_pillar_data` longtext DEFAULT NULL,
  `cp10_span_data` longtext DEFAULT NULL,
  `cp11_id` varchar(20) DEFAULT NULL,
  `cp11_foundation_data` longtext DEFAULT NULL,
  `cp11_base_data` longtext DEFAULT NULL,
  `cp11_pillar_data` longtext DEFAULT NULL,
  `cp11_span_data` longtext DEFAULT NULL,
  `cp12_id` varchar(20) DEFAULT NULL,
  `cp12_foundation_data` longtext DEFAULT NULL,
  `cp12_base_data` longtext DEFAULT NULL,
  `cp12_pillar_data` longtext DEFAULT NULL,
  `cp12_span_data` longtext DEFAULT NULL,
  `cp13_id` varchar(20) DEFAULT NULL,
  `cp13_foundation_data` longtext DEFAULT NULL,
  `cp13_base_data` longtext DEFAULT NULL,
  `cp13_pillar_data` longtext DEFAULT NULL,
  `cp13_span_data` longtext DEFAULT NULL,
  `cp14_id` varchar(20) DEFAULT NULL,
  `cp14_foundation_data` longtext DEFAULT NULL,
  `cp14_base_data` longtext DEFAULT NULL,
  `cp14_pillar_data` longtext DEFAULT NULL,
  `cp14_span_data` longtext DEFAULT NULL,
  `cp15_id` varchar(20) DEFAULT NULL,
  `cp15_foundation_data` longtext DEFAULT NULL,
  `cp15_base_data` longtext DEFAULT NULL,
  `cp15_pillar_data` longtext DEFAULT NULL,
  `cp15_span_data` longtext DEFAULT NULL,
  `road_deck1_id` varchar(20) DEFAULT NULL,
  `road_deck1_data` longtext DEFAULT NULL,
  `road_deck1_construction_points` varchar(50) DEFAULT NULL COMMENT 'construction_pointX = construction_pointY',
  `road_deck2_id` varchar(20) DEFAULT NULL,
  `road_deck2_data` longtext DEFAULT NULL,
  `road_deck2_construction_points` varchar(50) DEFAULT NULL,
  `road_deck3_id` varchar(20) DEFAULT NULL,
  `road_deck3_data` longtext DEFAULT NULL,
  `road_deck3_construction_points` varchar(50) DEFAULT NULL,
  `road_deck4_id` varchar(20) DEFAULT NULL,
  `road_deck4_data` longtext DEFAULT NULL,
  `road_deck4_construction_points` varchar(50) DEFAULT NULL,
  `road_deck5_id` varchar(20) DEFAULT NULL,
  `road_deck5_data` longtext DEFAULT NULL,
  `road_deck5_construction_points` varchar(50) DEFAULT NULL,
  `road_deck6_id` varchar(20) DEFAULT NULL,
  `road_deck6_data` longtext DEFAULT NULL,
  `road_deck6_construction_points` varchar(50) DEFAULT NULL,
  `road_deck7_id` varchar(20) DEFAULT NULL,
  `road_deck7_data` longtext DEFAULT NULL,
  `road_deck7_construction_points` varchar(50) DEFAULT NULL,
  `road_deck8_id` varchar(20) DEFAULT NULL,
  `road_deck8_data` longtext DEFAULT NULL,
  `road_deck8_construction_points` varchar(50) DEFAULT NULL,
  `road_deck9_id` varchar(20) DEFAULT NULL,
  `road_deck9_data` longtext DEFAULT NULL,
  `road_deck9_construction_points` varchar(50) DEFAULT NULL,
  `road_deck10_id` varchar(20) DEFAULT NULL,
  `road_deck10_data` longtext DEFAULT NULL,
  `road_deck10_construction_points` varchar(50) DEFAULT NULL,
  `road_deck11_id` varchar(20) DEFAULT NULL,
  `road_deck11_data` longtext DEFAULT NULL,
  `road_deck11_construction_points` varchar(50) DEFAULT NULL,
  `road_deck12_id` varchar(20) DEFAULT NULL,
  `road_deck12_data` longtext DEFAULT NULL,
  `road_deck12_construction_points` varchar(50) DEFAULT NULL,
  `road_deck13_id` varchar(20) DEFAULT NULL,
  `road_deck13_data` longtext DEFAULT NULL,
  `road_deck13_construction_points` varchar(50) DEFAULT NULL,
  `road_deck14_id` varchar(20) DEFAULT NULL,
  `road_deck14_data` longtext DEFAULT NULL,
  `road_deck14_construction_points` varchar(50) DEFAULT NULL,
  `road_deck15_id` varchar(20) DEFAULT NULL,
  `road_deck15_data` longtext DEFAULT NULL,
  `road_deck15_construction_points` varchar(50) DEFAULT NULL,
  `inserted_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `chainage_header_all`
--

CREATE TABLE `chainage_header_all` (
  `cha_id` varchar(20) NOT NULL,
  `cha_code` varchar(20) DEFAULT NULL,
  `cha_start_gps` longtext DEFAULT NULL,
  `cha_end_gps` longtext DEFAULT NULL,
  `cha_inserted_on` datetime NOT NULL,
  `kgcd_id` varchar(20) DEFAULT NULL,
  `tdfd_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contractor_doc_data_all`
--

CREATE TABLE `contractor_doc_data_all` (
  `cdd_id` varchar(20) NOT NULL,
  `cpc_id` varchar(25) NOT NULL COMMENT 'contractor_project_combination_all',
  `cdd_description` longtext DEFAULT NULL,
  `cdd_file_1` varchar(500) DEFAULT NULL,
  `cdd_file_2` varchar(500) DEFAULT NULL,
  `cdd_file_3` varchar(500) DEFAULT NULL,
  `cdd_file_4` varchar(500) DEFAULT NULL,
  `cdd_file_5` varchar(500) DEFAULT NULL,
  `cdd_file_6` varchar(500) DEFAULT NULL,
  `cdd_file_7` varchar(500) DEFAULT NULL,
  `cdd_file_8` varchar(500) DEFAULT NULL,
  `cdd_file_9` varchar(500) DEFAULT NULL,
  `cdd_file_10` varchar(500) DEFAULT NULL,
  `cdd_file_11` varchar(500) DEFAULT NULL,
  `cdd_file_12` varchar(500) DEFAULT NULL,
  `cdd_file_13` varchar(500) DEFAULT NULL,
  `cdd_file_14` varchar(500) DEFAULT NULL,
  `cdd_file_15` varchar(500) DEFAULT NULL,
  `cdd_file_16` varchar(500) DEFAULT NULL,
  `cdd_file_17` varchar(500) DEFAULT NULL,
  `cdd_file_18` varchar(500) DEFAULT NULL,
  `cdd_file_19` varchar(500) DEFAULT NULL,
  `cdd_file_20` varchar(500) DEFAULT NULL,
  `inserted_on` datetime NOT NULL,
  `cdoc_id` varchar(20) NOT NULL,
  `line_no` int(2) NOT NULL,
  `last_filled_column` int(2) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contractor_doc_header_all`
--

CREATE TABLE `contractor_doc_header_all` (
  `cdoc_id` varchar(20) NOT NULL,
  `cdoc_name` varchar(255) DEFAULT NULL,
  `cdoc_type` int(11) NOT NULL,
  `cdoc_status` int(11) NOT NULL,
  `cdoc_inserted_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contractor_header_all`
--

CREATE TABLE `contractor_header_all` (
  `coh_id` varchar(20) NOT NULL,
  `contractor_name` varchar(255) NOT NULL COMMENT 'Firm name on which contracts are taken',
  `coh_address` longtext DEFAULT NULL,
  `coh_city` varchar(25) NOT NULL,
  `coh_email` varchar(25) NOT NULL,
  `coh_mobile_no` varchar(10) NOT NULL,
  `status` int(1) NOT NULL COMMENT '1=Active, 0=Deactive',
  `inserted_on` datetime NOT NULL,
  `deactivated_on` datetime DEFAULT NULL,
  `attached_departmen` varchar(100) NOT NULL COMMENT 'In csv active departments ids where this contractor is working',
  `deactivated_by` varchar(25) NOT NULL COMMENT 'user id who only belongs to platform'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contractor_project_combination_all`
--

CREATE TABLE `contractor_project_combination_all` (
  `cpc_id` varchar(25) NOT NULL,
  `coh_id` varchar(25) NOT NULL COMMENT 'contractor_header_all',
  `prj_id` varchar(25) NOT NULL COMMENT 'project_header_all',
  `pac_id` varchar(25) NOT NULL COMMENT 'May be blank. if contractor on entire project',
  `dh_id` varchar(25) NOT NULL COMMENT 'department_header_all',
  `from_km_chainage` int(10) NOT NULL,
  `to_km_chainage` int(10) NOT NULL,
  `contract_period` int(10) NOT NULL COMMENT 'define in months',
  `type_of_contract` int(1) NOT NULL COMMENT '1= Survey, 2=Road construction, 3=Bridge specific, 4=Electrical Instrumentations, 5=Toll Booth, 6=Poll Fixing, 7=Tunnling, 8=Painting',
  `contract_number` int(11) NOT NULL,
  `status_of_contract` int(1) NOT NULL COMMENT '1= Awarded, 2=Operational, 3=completed, 4=Terminated',
  `inserted_on` datetime NOT NULL,
  `started_on` date NOT NULL,
  `will_complete_on` date NOT NULL,
  `status` int(1) NOT NULL COMMENT '1=Active, 2=Deactive',
  `contract_association` int(1) NOT NULL COMMENT '1=main contractor, 2=sub contractor',
  `principal_coh_id` varchar(25) NOT NULL COMMENT 'In case os sub contractor principle id will be there.'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contract_milestone_header_all`
--

CREATE TABLE `contract_milestone_header_all` (
  `mha_id` varchar(20) NOT NULL,
  `mha_name` varchar(255) DEFAULT NULL,
  `milestone_no` int(3) NOT NULL,
  `mha_tenure_days` varchar(50) DEFAULT NULL,
  `mha_inserted_on` datetime NOT NULL,
  `mha_updated_on` datetime NOT NULL,
  `cpc_id` varchar(25) NOT NULL COMMENT 'contractor_project_combination_all',
  `contract_type` int(1) NOT NULL COMMENT '1=Road, 2=Bridge, 3=Building, 4=All',
  `inserted_by` varchar(25) NOT NULL COMMENT 'emp_id who is managing him'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `contract_work_stages_details_all`
--

CREATE TABLE `contract_work_stages_details_all` (
  `cws_id` varchar(25) NOT NULL,
  `cms_id` varchar(25) NOT NULL COMMENT 'contract_milestone_header_all',
  `stage_detail` varchar(200) NOT NULL,
  `from_km` int(11) NOT NULL,
  `from_chainage` int(11) NOT NULL,
  `to_km` int(11) NOT NULL,
  `to_chainage` int(11) NOT NULL,
  `completion_days` int(11) NOT NULL,
  `sequence_no` int(11) NOT NULL COMMENT 'Stages has a sequence systematic ',
  `stage_activity` int(11) NOT NULL,
  `stage_type` int(1) NOT NULL COMMENT '1=Road, 2=Bridge, 3=Building (only one is permitted)'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `department_header_all`
--

CREATE TABLE `department_header_all` (
  `dh_id` varchar(20) NOT NULL,
  `dept_type` int(11) NOT NULL COMMENT '1=Govt dept, 2= private agency',
  `dh_name` varchar(100) NOT NULL,
  `dh_full_name` varchar(100) NOT NULL,
  `dh_logo` varchar(100) DEFAULT NULL,
  `active_projects` int(11) NOT NULL COMMENT 'No of projects which is currently operational',
  `register_contracters` int(11) DEFAULT NULL COMMENT 'Associated contractors',
  `status` int(11) NOT NULL COMMENT '1 = Active\r\n0 = Non active',
  `dh_login_url` varchar(200) NOT NULL,
  `dh_login_file_type` int(11) DEFAULT NULL,
  `dh_login_file` varchar(100) DEFAULT NULL,
  `dh_dashboard_file_type` int(11) DEFAULT NULL,
  `dh_dashboard_file` varchar(100) DEFAULT NULL,
  `dh_tps` int(11) DEFAULT NULL,
  `dh_tps_file` varchar(100) DEFAULT NULL,
  `dh_tps_old_files` longtext DEFAULT NULL,
  `inserted_on` datetime NOT NULL,
  `deactivated_on` datetime NOT NULL,
  `deactivated_by` varchar(25) NOT NULL COMMENT 'user_id who did the deactivation',
  `top_authority_id` varchar(25) NOT NULL COMMENT 'user_id who is the big boss(dept head)',
  `top_authority_from` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `district_header_all`
--

CREATE TABLE `district_header_all` (
  `dist_id` varchar(20) NOT NULL,
  `dist_name` varchar(100) NOT NULL,
  `dist_geojson_file` longtext DEFAULT NULL,
  `dist_inserted_on` datetime NOT NULL,
  `dist_status` int(11) NOT NULL,
  `st_id` varchar(20) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `emp_area_jurisdiction_details_all`
--

CREATE TABLE `emp_area_jurisdiction_details_all` (
  `emp_id` varchar(25) NOT NULL,
  `dh_id` varchar(25) NOT NULL COMMENT 'single district',
  `dt_id` varchar(25) NOT NULL COMMENT 'multiple taluka in csv.\r\nif district is define but taluka is blank than particular emp controlling entire taluka and their villages.',
  `tl_id` longtext NOT NULL COMMENT 'multiple villages of all the taluka defines in csv.\r\nif taluka is define but village is blank than particular emp controlling entire taluka villages.',
  `vil_id` longtext NOT NULL,
  `inserted_on` datetime NOT NULL,
  `valid_till` datetime NOT NULL,
  `line_no` int(1) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `emp_header_all`
--

CREATE TABLE `emp_header_all` (
  `emp_id` varchar(25) NOT NULL,
  `primary_id` varchar(25) NOT NULL COMMENT 'dh_id, cont_id, scoh_id."It may be blank in case of platform user"',
  `primary_type` int(1) NOT NULL COMMENT '1=platform, 2=department, 3= contractor, 4 = sub contractor',
  `emp_name` varchar(50) NOT NULL COMMENT 'Fisrt Middle Last Name',
  `emp_category` int(1) NOT NULL COMMENT '1=authority, 2=operational incharge, 3=management, 4=on site, 5=main authority',
  `status` int(1) NOT NULL COMMENT '1 = active, 2 = deactive',
  `user_id` varchar(25) NOT NULL,
  `dh_id` varchar(25) NOT NULL COMMENT 'department_header_all',
  `ph_id` varchar(25) NOT NULL COMMENT 'project_header_all',
  `coh_id` varchar(25) NOT NULL COMMENT 'contractor_header_all',
  `inserted_on` datetime NOT NULL,
  `deactivated_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `emp_profile_allocation_all`
--

CREATE TABLE `emp_profile_allocation_all` (
  `epa_id` int(11) NOT NULL,
  `emp_id` int(11) NOT NULL,
  `emp_type` int(11) NOT NULL,
  `created_by` int(11) NOT NULL COMMENT '1=platform,2= dept, 3= contractor who create this profile',
  `creater_type` int(11) NOT NULL COMMENT '1=platform,2= dept, 3= contractor',
  `line_no` int(11) NOT NULL,
  `inserted_on` int(11) NOT NULL,
  `valid_till` int(11) NOT NULL,
  `pro_id_1` int(11) NOT NULL,
  `pro_id_2` varchar(25) NOT NULL,
  `pro_id_3` varchar(25) NOT NULL,
  `pro_id_4` varchar(25) NOT NULL,
  `pro_id_5` varchar(25) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `emp_project_authority_all`
--

CREATE TABLE `emp_project_authority_all` (
  `epa_id` varchar(25) NOT NULL,
  `emp_id` varchar(25) NOT NULL COMMENT 'emp_header_all',
  `dh_id` varchar(25) NOT NULL COMMENT 'department_header_all',
  `ph_id` varchar(25) NOT NULL COMMENT 'project_header_all',
  `pac_id` varchar(25) NOT NULL COMMENT 'package_header_all',
  `cpc_id` varchar(25) NOT NULL COMMENT 'contarctor_project_combination_all',
  `pro_id` longtext NOT NULL COMMENT 'profile ids in csv',
  `Authority 1` varchar(255) NOT NULL,
  `Authority 2` varchar(255) NOT NULL,
  `Authority 3` varchar(255) NOT NULL,
  `Authority 4` varchar(255) NOT NULL,
  `Authority 5` varchar(255) NOT NULL,
  `Authority 6` varchar(255) NOT NULL,
  `Authority 7` varchar(255) NOT NULL,
  `Authority 8` varchar(255) NOT NULL,
  `Authority 9` varchar(255) NOT NULL,
  `Authority 10` varchar(255) NOT NULL,
  `Authority 11` varchar(255) NOT NULL,
  `Authority 12` varchar(255) NOT NULL,
  `Authority 13` varchar(255) NOT NULL,
  `Authority 14` varchar(255) NOT NULL,
  `Authority 15` varchar(255) NOT NULL,
  `Authority 16` varchar(255) NOT NULL,
  `Authority 17` varchar(255) NOT NULL,
  `Authority 18` varchar(255) NOT NULL,
  `Authority 19` varchar(255) NOT NULL,
  `Authority 20` varchar(255) NOT NULL,
  `Authority 21` varchar(255) NOT NULL,
  `Authority 22` varchar(255) NOT NULL,
  `Authority 23` varchar(255) NOT NULL,
  `Authority 24` varchar(255) NOT NULL,
  `Authority 25` varchar(255) NOT NULL,
  `Authority 26` varchar(255) NOT NULL,
  `Authority 27` varchar(255) NOT NULL,
  `Authority 28` varchar(255) NOT NULL,
  `Authority 29` varchar(255) NOT NULL,
  `Authority 30` varchar(255) NOT NULL,
  `Authority 31` varchar(255) NOT NULL,
  `line_on` int(2) NOT NULL,
  `inserted_on` datetime NOT NULL,
  `valid_till` date NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `emp_project_jurisdiction_details_all`
--

CREATE TABLE `emp_project_jurisdiction_details_all` (
  `epj_id` varchar(25) NOT NULL,
  `emp_id` varchar(25) NOT NULL COMMENT 'emp_header_all',
  `dh_id` varchar(25) NOT NULL COMMENT 'department_header_all',
  `belongs_to` int(1) NOT NULL COMMENT '1=platform(not applicable), 2=department, 3= contractor, 4 = sub contractor	',
  `ph_id` varchar(25) NOT NULL COMMENT 'project_header_all',
  `pac_id` varchar(25) NOT NULL,
  `cpc_id` varchar(25) NOT NULL COMMENT 'contractor employee',
  `from_km` int(10) NOT NULL,
  `from_chainage` int(10) NOT NULL,
  `to_km` int(10) NOT NULL,
  `to_chainage` int(10) NOT NULL,
  `line_no` int(1) NOT NULL,
  `inserted_on` datetime NOT NULL,
  `valid_till` date NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `first_sub_folder_details_all`
--

CREATE TABLE `first_sub_folder_details_all` (
  `first_id` varchar(20) NOT NULL,
  `first_folder_name` varchar(100) NOT NULL,
  `pac_id` varchar(20) DEFAULT NULL,
  `tdoc_id` varchar(20) NOT NULL,
  `t_pdc_id` varchar(20) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `km_gps_coordinate_details_all`
--

CREATE TABLE `km_gps_coordinate_details_all` (
  `kgcd_id` varchar(20) NOT NULL,
  `kgcd_km` varchar(20) DEFAULT NULL,
  `kgcd_start_gps` varchar(200) DEFAULT NULL,
  `kgcd_end_gps` varchar(200) DEFAULT NULL,
  `kgcd_inserted_on` datetime NOT NULL,
  `end_dist_id` varchar(20) DEFAULT NULL,
  `st_dist_id` varchar(20) DEFAULT NULL,
  `ph_id` varchar(20) DEFAULT NULL,
  `end_tal_id` varchar(20) DEFAULT NULL,
  `st_tal_id` varchar(20) DEFAULT NULL,
  `tdfd_id` varchar(20) DEFAULT NULL,
  `end_vil_id` varchar(20) DEFAULT NULL,
  `st_vil_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `layer_cross_reference_details_all`
--

CREATE TABLE `layer_cross_reference_details_all` (
  `lcrd_id` int(25) NOT NULL,
  `parent_layer_id` varchar(25) NOT NULL,
  `parent_ref_line_name` varchar(25) NOT NULL COMMENT 'reference of design layer',
  `child_layer_id` varchar(25) NOT NULL,
  `status` int(1) NOT NULL COMMENT '1=Linked, 2=Delinked',
  `inserted_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `measurement_layer_details_all`
--

CREATE TABLE `measurement_layer_details_all` (
  `lmd_id` varchar(255) NOT NULL,
  `worksheet_id` varchar(25) DEFAULT NULL COMMENT 'project_worksheet_header_all',
  `layer_id` varchar(20) DEFAULT NULL COMMENT 'worksheet_layers_header_all',
  `layer_type` tinyint(1) DEFAULT NULL COMMENT '1=Cutting, 2=Filling',
  `ref_layer_id` varchar(20) DEFAULT NULL,
  `ref_layer_line` tinyint(1) DEFAULT NULL COMMENT '1=Construction, 2=Road surface, 3=Surface',
  `line_on` varchar(50) DEFAULT NULL,
  `1_from_km` int(11) DEFAULT NULL,
  `1_to_km` int(11) DEFAULT NULL,
  `1_from_chainage` int(11) DEFAULT NULL,
  `1_to_chainage` int(11) DEFAULT NULL,
  `2_from_km` int(11) DEFAULT NULL,
  `2_to_km` int(11) DEFAULT NULL,
  `2_from_chainage` int(11) DEFAULT NULL,
  `2_to_chainage` int(11) DEFAULT NULL,
  `3_from_km` int(11) DEFAULT NULL,
  `3_to_km` int(11) DEFAULT NULL,
  `3_from_chainage` int(11) DEFAULT NULL,
  `3_to_chainage` int(11) DEFAULT NULL,
  `4_from_km` int(11) DEFAULT NULL,
  `4_to_km` int(11) DEFAULT NULL,
  `4_from_chainage` int(11) DEFAULT NULL,
  `4_to_chainage` int(11) DEFAULT NULL,
  `5_from_km` int(11) DEFAULT NULL,
  `5_to_km` int(11) DEFAULT NULL,
  `5_from_chainage` int(11) DEFAULT NULL,
  `5_to_chainage` int(11) DEFAULT NULL,
  `6_from_km` int(11) DEFAULT NULL,
  `6_to_km` int(11) DEFAULT NULL,
  `6_from_chainage` int(11) DEFAULT NULL,
  `6_to_chainage` int(11) DEFAULT NULL,
  `7_from_km` int(11) DEFAULT NULL,
  `7_to_km` int(11) DEFAULT NULL,
  `7_from_chainage` int(11) DEFAULT NULL,
  `7_to_chainage` int(11) DEFAULT NULL,
  `8_from_km` int(11) DEFAULT NULL,
  `8_to_km` int(11) DEFAULT NULL,
  `8_from_chainage` int(11) DEFAULT NULL,
  `8_to_chainage` int(11) DEFAULT NULL,
  `9_from_km` int(11) DEFAULT NULL,
  `9_to_km` int(11) DEFAULT NULL,
  `9_from_chainage` int(11) DEFAULT NULL,
  `9_to_chainage` int(11) DEFAULT NULL,
  `10_from_km` int(11) DEFAULT NULL,
  `10_to_km` int(11) DEFAULT NULL,
  `10_from_chainage` int(11) DEFAULT NULL,
  `10_to_chainage` int(11) DEFAULT NULL,
  `11_from_km` int(11) DEFAULT NULL,
  `11_to_km` int(11) DEFAULT NULL,
  `11_from_chainage` int(11) DEFAULT NULL,
  `11_to_chainage` int(11) DEFAULT NULL,
  `12_from_km` int(11) DEFAULT NULL,
  `12_to_km` int(11) DEFAULT NULL,
  `12_from_chainage` int(11) DEFAULT NULL,
  `12_to_chainage` int(11) DEFAULT NULL,
  `13_from_km` int(11) DEFAULT NULL,
  `13_to_km` int(11) DEFAULT NULL,
  `13_from_chainage` int(11) DEFAULT NULL,
  `13_to_chainage` int(11) DEFAULT NULL,
  `14_from_km` int(11) DEFAULT NULL,
  `14_to_km` int(11) DEFAULT NULL,
  `14_from_chainage` int(11) DEFAULT NULL,
  `14_to_chainage` int(11) DEFAULT NULL,
  `15_from_km` int(11) DEFAULT NULL,
  `15_to_km` int(11) DEFAULT NULL,
  `15_from_chainage` int(11) DEFAULT NULL,
  `15_to_chainage` int(11) DEFAULT NULL,
  `16_from_km` int(11) DEFAULT NULL,
  `16_to_km` int(11) DEFAULT NULL,
  `16_from_chainage` int(11) DEFAULT NULL,
  `16_to_chainage` int(11) DEFAULT NULL,
  `17_from_km` int(11) DEFAULT NULL,
  `17_to_km` int(11) DEFAULT NULL,
  `17_from_chainage` int(11) DEFAULT NULL,
  `17_to_chainage` int(11) DEFAULT NULL,
  `18_from_km` int(11) DEFAULT NULL,
  `18_to_km` int(11) DEFAULT NULL,
  `18_from_chainage` int(11) DEFAULT NULL,
  `18_to_chainage` int(11) DEFAULT NULL,
  `19_from_km` int(11) DEFAULT NULL,
  `19_to_km` int(11) DEFAULT NULL,
  `19_from_chainage` int(11) DEFAULT NULL,
  `19_to_chainage` int(11) DEFAULT NULL,
  `20_from_km` int(11) DEFAULT NULL,
  `20_to_km` int(11) DEFAULT NULL,
  `20_from_chainage` int(11) DEFAULT NULL,
  `20_to_chainage` int(11) DEFAULT NULL,
  `1_qty` decimal(10,2) DEFAULT NULL COMMENT 'if filling then filling qty, if cutting then cutting qty',
  `2_qty` decimal(10,2) DEFAULT NULL,
  `3_qty` decimal(10,2) DEFAULT NULL,
  `4_qty` decimal(10,2) DEFAULT NULL,
  `5_qty` decimal(10,2) DEFAULT NULL,
  `6_qty` decimal(10,2) DEFAULT NULL,
  `7_qty` decimal(10,2) DEFAULT NULL,
  `8_qty` decimal(10,2) DEFAULT NULL,
  `9_qty` decimal(10,2) DEFAULT NULL,
  `10_qty` decimal(10,2) DEFAULT NULL,
  `11_qty` decimal(10,2) DEFAULT NULL,
  `12_qty` decimal(10,2) DEFAULT NULL,
  `13_qty` decimal(10,2) DEFAULT NULL,
  `14_qty` decimal(10,2) DEFAULT NULL,
  `15_qty` decimal(10,2) DEFAULT NULL,
  `16_qty` decimal(10,2) DEFAULT NULL,
  `17_qty` decimal(10,2) DEFAULT NULL,
  `18_qty` decimal(10,2) DEFAULT NULL,
  `19_qty` decimal(10,2) DEFAULT NULL,
  `20_qty` decimal(10,2) DEFAULT NULL,
  `1_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `1_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `2_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `2_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `3_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `3_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `4_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `4_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `5_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `5_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `6_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `6_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `7_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `7_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `8_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `8_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `9_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `9_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `10_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `10_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `11_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `11_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `12_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `12_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `13_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `13_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `14_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `14_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `15_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `15_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `16_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `16_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `17_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `17_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `18_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `18_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `19_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `19_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL,
  `20_Surface_to_construction_line` varchar(50) DEFAULT NULL,
  `20_Surface_to_roadsurface_line` varchar(50) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `package_header_all`
--

CREATE TABLE `package_header_all` (
  `pac_id` varchar(20) NOT NULL,
  `pac_name` longtext DEFAULT NULL,
  `pac_no` longtext DEFAULT NULL,
  `pac_type` int(11) DEFAULT NULL COMMENT '1=Land Acquisition, 2= JM(Joint Measurment), 3=Construction, 4=Completed, 5=Maintenance',
  `pac_from_km` varchar(20) DEFAULT NULL,
  `from_chainage` int(10) NOT NULL,
  `pac_to_km` varchar(20) DEFAULT NULL,
  `to_chainage` int(10) NOT NULL,
  `inserted_on` datetime NOT NULL,
  `status` int(1) NOT NULL COMMENT '1=active, 2=deactive',
  `deactivated_on` datetime DEFAULT NULL,
  `ph_id` varchar(20) DEFAULT NULL COMMENT 'project id'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `pano_files_details_all`
--

CREATE TABLE `pano_files_details_all` (
  `pano_id` varchar(15) NOT NULL,
  `pfd_id` varchar(20) DEFAULT NULL,
  `pfd_line_no` int(11) NOT NULL,
  `pfd_curr_col_no` int(11) NOT NULL,
  `para_id_1` varchar(20) DEFAULT NULL,
  `para_id_1_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_1_details`)),
  `para_id_2` varchar(20) DEFAULT NULL,
  `para_id_2_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_2_details`)),
  `para_id_3` varchar(20) DEFAULT NULL,
  `para_id_3_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_3_details`)),
  `para_id_4` varchar(20) DEFAULT NULL,
  `para_id_4_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_4_details`)),
  `para_id_5` varchar(20) DEFAULT NULL,
  `para_id_5_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_5_details`)),
  `para_id_6` varchar(20) DEFAULT NULL,
  `para_id_6_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_6_details`)),
  `para_id_7` varchar(20) DEFAULT NULL,
  `para_id_7_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_7_details`)),
  `para_id_8` varchar(20) DEFAULT NULL,
  `para_id_8_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_8_details`)),
  `para_id_9` varchar(20) DEFAULT NULL,
  `para_id_9_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_9_details`)),
  `para_id_10` varchar(20) DEFAULT NULL,
  `para_id_10_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_10_details`)),
  `para_id_11` varchar(20) DEFAULT NULL,
  `para_id_11_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_11_details`)),
  `para_id_12` varchar(20) DEFAULT NULL,
  `para_id_12_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_12_details`)),
  `para_id_13` varchar(20) DEFAULT NULL,
  `para_id_13_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_13_details`)),
  `para_id_14` varchar(20) DEFAULT NULL,
  `para_id_14_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_14_details`)),
  `para_id_15` varchar(20) DEFAULT NULL,
  `para_id_15_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_15_details`)),
  `para_id_16` varchar(20) DEFAULT NULL,
  `para_id_16_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_16_details`)),
  `para_id_17` varchar(20) DEFAULT NULL,
  `para_id_17_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_17_details`)),
  `para_id_18` varchar(20) DEFAULT NULL,
  `para_id_18_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_18_details`)),
  `para_id_19` varchar(20) DEFAULT NULL,
  `para_id_19_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_19_details`)),
  `para_id_20` varchar(20) DEFAULT NULL,
  `para_id_20_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_20_details`)),
  `para_id_21` varchar(20) DEFAULT NULL,
  `para_id_21_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_21_details`)),
  `para_id_22` varchar(20) DEFAULT NULL,
  `para_id_22_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_22_details`)),
  `para_id_23` varchar(20) DEFAULT NULL,
  `para_id_23_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_23_details`)),
  `para_id_24` varchar(20) DEFAULT NULL,
  `para_id_24_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_24_details`)),
  `para_id_25` varchar(20) DEFAULT NULL,
  `para_id_25_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_25_details`)),
  `para_id_26` varchar(20) DEFAULT NULL,
  `para_id_26_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_26_details`)),
  `para_id_27` varchar(20) DEFAULT NULL,
  `para_id_27_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_27_details`)),
  `para_id_28` varchar(20) DEFAULT NULL,
  `para_id_28_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_28_details`)),
  `para_id_29` varchar(20) DEFAULT NULL,
  `para_id_29_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_29_details`)),
  `para_id_30` varchar(20) DEFAULT NULL,
  `para_id_30_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_30_details`)),
  `para_id_31` varchar(20) DEFAULT NULL,
  `para_id_31_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_31_details`)),
  `para_id_32` varchar(20) DEFAULT NULL,
  `para_id_32_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_32_details`)),
  `para_id_33` varchar(20) DEFAULT NULL,
  `para_id_33_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_33_details`)),
  `para_id_34` varchar(20) DEFAULT NULL,
  `para_id_34_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_34_details`)),
  `para_id_35` varchar(20) DEFAULT NULL,
  `para_id_35_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_35_details`)),
  `para_id_36` varchar(20) DEFAULT NULL,
  `para_id_36_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_36_details`)),
  `para_id_37` varchar(20) DEFAULT NULL,
  `para_id_37_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_37_details`)),
  `para_id_38` varchar(20) DEFAULT NULL,
  `para_id_38_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_38_details`)),
  `para_id_39` varchar(20) DEFAULT NULL,
  `para_id_39_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_39_details`)),
  `para_id_40` varchar(20) DEFAULT NULL,
  `para_id_40_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_40_details`)),
  `para_id_41` varchar(20) DEFAULT NULL,
  `para_id_41_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_41_details`)),
  `para_id_42` varchar(20) DEFAULT NULL,
  `para_id_42_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_42_details`)),
  `para_id_43` varchar(20) DEFAULT NULL,
  `para_id_43_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_43_details`)),
  `para_id_44` varchar(20) DEFAULT NULL,
  `para_id_44_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_44_details`)),
  `para_id_45` varchar(20) DEFAULT NULL,
  `para_id_45_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_45_details`)),
  `para_id_46` varchar(20) DEFAULT NULL,
  `para_id_46_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_46_details`)),
  `para_id_47` varchar(20) DEFAULT NULL,
  `para_id_47_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_47_details`)),
  `para_id_48` varchar(20) DEFAULT NULL,
  `para_id_48_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_48_details`)),
  `para_id_49` varchar(20) DEFAULT NULL,
  `para_id_49_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_49_details`)),
  `para_id_50` varchar(20) DEFAULT NULL,
  `para_id_50_details` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`para_id_50_details`)),
  `pfd_inserted_on` datetime NOT NULL,
  `pfd_status` int(11) NOT NULL,
  `kgcd_id` varchar(20) DEFAULT NULL,
  `pdh_id` varchar(20) DEFAULT NULL,
  `ph_id` varchar(20) DEFAULT NULL,
  `dch_id` varchar(20) DEFAULT NULL,
  `cha_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `para_direction_header_all`
--

CREATE TABLE `para_direction_header_all` (
  `pdh_id` varchar(20) NOT NULL,
  `pdh_direction` longtext DEFAULT NULL,
  `pdh_inserted_on` datetime NOT NULL,
  `ph_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `profile_header_all`
--

CREATE TABLE `profile_header_all` (
  `pro_id` varchar(20) NOT NULL,
  `pro_name` varchar(100) NOT NULL,
  `pro_form_ids` varchar(100) DEFAULT NULL,
  `pro_process_ids` varchar(100) DEFAULT NULL,
  `pro_status` int(11) NOT NULL,
  `pro_inserted_on` datetime NOT NULL,
  `pro_deactivated_on` datetime DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `project_header_all`
--

CREATE TABLE `project_header_all` (
  `ph_id` varchar(20) NOT NULL,
  `ph_name` varchar(100) NOT NULL,
  `ph_description` longtext DEFAULT NULL,
  `project_km_length` int(20) DEFAULT NULL COMMENT 'Total length in KM',
  `ph_from_km` varchar(20) DEFAULT NULL,
  `ph_to_km` varchar(20) DEFAULT NULL,
  `ph_project_image` varchar(100) DEFAULT NULL,
  `ph_type` int(1) NOT NULL COMMENT '1=Road, 2=TPS, 3= BUilding',
  `ph_tps_type` int(11) DEFAULT NULL,
  `ph_tps_area` varchar(50) DEFAULT NULL,
  `ph_tps_village_count` int(11) DEFAULT NULL,
  `ph_status` int(11) NOT NULL,
  `ph_file_type` int(11) DEFAULT NULL,
  `ph_file` varchar(100) DEFAULT NULL,
  `ph_old_files` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`ph_old_files`)),
  `dh_id` varchar(20) DEFAULT NULL,
  `ph_stage` int(1) DEFAULT NULL COMMENT '1=Land Acquisition, 2= JM(Joint Measurment), 3=Construction, 4=Completed, 5=Maintenance'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `project_location_details_all`
--

CREATE TABLE `project_location_details_all` (
  `pld_id` varchar(20) NOT NULL,
  `pld_location_type` int(11) NOT NULL,
  `pld_vil_code` varchar(3) NOT NULL,
  `pld_area` varchar(20) NOT NULL,
  `pld_area_measurement_type` varchar(20) DEFAULT NULL,
  `vil_total_private_land` varchar(20) DEFAULT NULL,
  `vil_private_land_measurement_type` varchar(20) DEFAULT NULL,
  `vil_total_govt_land` varchar(20) DEFAULT NULL,
  `vil_govt_land_measurement_type` varchar(20) DEFAULT NULL,
  `vil_total_forest_land` varchar(20) DEFAULT NULL,
  `vil_forest_land_measurement_type` varchar(20) DEFAULT NULL,
  `vil_total_other_land` varchar(20) DEFAULT NULL,
  `vil_other_land_measurement_type` varchar(20) DEFAULT NULL,
  `pld_status` tinyint(1) NOT NULL,
  `pld_aq_area` varchar(255) DEFAULT NULL,
  `pld_start_km` varchar(20) NOT NULL,
  `pld_end_km` varchar(20) NOT NULL,
  `pld_file_type` int(11) DEFAULT NULL,
  `pld_aqu_land_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_aqu_land_file`)),
  `pld_vil_boundry_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_vil_boundry_file`)),
  `pld_survey_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_survey_file`)),
  `pld_hissa_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_hissa_file`)),
  `pld_op_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_op_file`)),
  `pld_fp_file` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_fp_file`)),
  `pld_old_files` longtext CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL CHECK (json_valid(`pld_old_files`)),
  `pld_inserted_on` datetime NOT NULL,
  `dist_id` varchar(20) DEFAULT NULL,
  `ph_id` varchar(20) DEFAULT NULL,
  `st_id` varchar(20) DEFAULT NULL,
  `tal_id` varchar(20) DEFAULT NULL,
  `vil_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `project_worksheet_header_all`
--

CREATE TABLE `project_worksheet_header_all` (
  `id` int(11) NOT NULL,
  `worksheet_id` varchar(255) NOT NULL,
  `ph_id` varchar(255) NOT NULL COMMENT 'project_header_all',
  `worksheet_name` varchar(255) NOT NULL,
  `worksheet_config_data` varchar(255) NOT NULL,
  `inserted_by` varchar(25) NOT NULL COMMENT 'desktop user id',
  `status` int(1) NOT NULL COMMENT '0=not in use, 1 = operational, 2=change suggested',
  `inserted_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `road_cycle_detail_all`
--

CREATE TABLE `road_cycle_detail_all` (
  `rcd_id` int(11) NOT NULL,
  `cycle_id` int(11) NOT NULL,
  `cycle_type` int(11) NOT NULL COMMENT '1=rolling, 2=watering',
  `eqp_id` int(11) NOT NULL,
  `line_no` int(11) NOT NULL COMMENT 'master entry',
  `eqp_operation_details` int(11) NOT NULL COMMENT 'json format',
  `cycle_seq_no` int(11) NOT NULL,
  `cyclic_count` int(11) NOT NULL COMMENT 'no of rotation'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `road_layers_data_details_all`
--

CREATE TABLE `road_layers_data_details_all` (
  `lrdd_id` int(25) NOT NULL,
  `worksheet_id` varchar(20) DEFAULT NULL COMMENT 'project_worksheet_header_all',
  `layer_id` varchar(20) DEFAULT NULL COMMENT 'worksheet_layer_header_all',
  `layer_type` tinyint(1) DEFAULT NULL COMMENT '1=Design, 2=Material',
  `from_km` int(11) DEFAULT NULL COMMENT '100',
  `from_chainage` int(11) DEFAULT NULL COMMENT '00 means is starting',
  `to_km` int(11) DEFAULT NULL COMMENT '100',
  `to_chainage` int(11) DEFAULT NULL COMMENT '05 means its ending',
  `surface_to_construction_line` float DEFAULT NULL COMMENT 'e.g. 10.4',
  `stc_type` int(11) NOT NULL COMMENT 'stc means surface to construction 1=digging, 2=cutting',
  `stc_approx_volume` double NOT NULL COMMENT 'qty in cubic in meter',
  `construction_to_road_surface_line` varchar(20) DEFAULT NULL COMMENT 'e.g. 02.45 mtr',
  `surface_to_construction_type` tinyint(1) DEFAULT NULL COMMENT '1=Digging, 2=Cutting, 3=Filling',
  `surface_line_width` varchar(20) DEFAULT NULL COMMENT 'e.g. 15 mtr',
  `construction_line_width` varchar(20) DEFAULT NULL COMMENT 'e.g. 12 mtr',
  `road_surface_line_width` varchar(20) DEFAULT NULL COMMENT 'e.g. 15 mtr',
  `road_angle` decimal(5,2) DEFAULT NULL COMMENT 'Angle in degrees',
  `angle_direction` tinyint(1) DEFAULT NULL COMMENT '1=Inner, 2=Outer, 3=No angle',
  `material1` varchar(20) DEFAULT NULL COMMENT 'Unique material number\r\nNote: material - 1 can be blank',
  `material1_name` varchar(100) DEFAULT NULL COMMENT 'Material name',
  `material1_details` varchar(255) NOT NULL COMMENT 'sub material name, id, qty, qty unit',
  `material1_ref_to` tinyint(1) DEFAULT NULL COMMENT 'layer_id > (1=Construction line, 2=Material). Material ID used only if value=2',
  `material1_ref_to_distance` varchar(20) DEFAULT NULL COMMENT 'xx.xx = yy.yy where xx.xx = filling and yy.yy = after rolling',
  `material1_qty` float NOT NULL COMMENT 'cubic meter qty of the perticular chainage\r\n',
  `material2` varchar(20) DEFAULT NULL,
  `material2_name` varchar(100) DEFAULT NULL,
  `material2_ref_to` tinyint(1) DEFAULT NULL,
  `material2_ref_to_distance` varchar(20) DEFAULT NULL COMMENT 'xx.xx=yy.yy \r\nxx means - filling height\r\nyy means - final compressed height',
  `material2_qty` float NOT NULL COMMENT 'cubic meter qty of the perticular chainage',
  `material3` varchar(20) DEFAULT NULL,
  `material3_name` varchar(100) DEFAULT NULL,
  `material3_ref_to` tinyint(1) DEFAULT NULL,
  `material3_ref_to_distance` varchar(20) DEFAULT NULL,
  `material4` varchar(20) DEFAULT NULL,
  `material4_name` varchar(100) DEFAULT NULL,
  `material4_ref_to` tinyint(1) DEFAULT NULL,
  `material4_ref_to_distance` varchar(20) DEFAULT NULL,
  `material5_id` varchar(20) DEFAULT NULL,
  `material5_name` varchar(100) DEFAULT NULL,
  `material5_ref_to` tinyint(1) DEFAULT NULL,
  `material5_ref_to_distance` varchar(20) DEFAULT NULL,
  `material6` varchar(20) DEFAULT NULL,
  `material6_name` varchar(100) DEFAULT NULL,
  `material6_ref_to` tinyint(1) DEFAULT NULL,
  `material6_ref_to_distance` varchar(20) DEFAULT NULL,
  `material7` varchar(20) DEFAULT NULL,
  `material7_name` varchar(100) DEFAULT NULL,
  `material7_ref_to` tinyint(1) DEFAULT NULL,
  `material7_ref_to_distance` varchar(20) DEFAULT NULL,
  `material8` varchar(20) DEFAULT NULL,
  `material8_name` varchar(100) DEFAULT NULL,
  `material8_ref_to` tinyint(1) DEFAULT NULL,
  `material8_ref_to_distance` varchar(20) DEFAULT NULL,
  `material9` varchar(20) DEFAULT NULL,
  `material9_name` varchar(100) DEFAULT NULL,
  `material9_ref_to` tinyint(1) DEFAULT NULL,
  `material9_ref_to_distance` varchar(20) DEFAULT NULL,
  `material10` varchar(20) DEFAULT NULL,
  `material10_name` varchar(100) DEFAULT NULL,
  `material10_ref_to` tinyint(1) DEFAULT NULL,
  `material10_ref_to_distance` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `road_step_gap_details_all`
--

CREATE TABLE `road_step_gap_details_all` (
  `rsgd_id` int(11) NOT NULL,
  `target_id` int(11) NOT NULL COMMENT 'target_header_all',
  `1st_step_id` int(11) NOT NULL,
  `2nd_step_id` int(11) NOT NULL,
  `gap_in_days` int(11) NOT NULL,
  `status` int(11) NOT NULL COMMENT '1=active, 2=modified',
  `line_no` int(11) NOT NULL,
  `inserted_on` datetime NOT NULL,
  `valid_till` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `road_target_header_all`
--

CREATE TABLE `road_target_header_all` (
  `trh_id` int(11) NOT NULL,
  `from_km` int(11) NOT NULL,
  `from_chainage` int(11) NOT NULL,
  `to_km` int(11) NOT NULL,
  `to_chainage` int(11) NOT NULL,
  `completion_in_days` int(11) NOT NULL,
  `trget_type` int(11) NOT NULL COMMENT '1=overall(main target), 2= sub target',
  `contractor_id` int(11) NOT NULL,
  `work_days` int(11) NOT NULL,
  `main_target` int(11) NOT NULL COMMENT 'empty if type is main\r\nif type is sub target then enter the main target'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `road_target_step_combination_all`
--

CREATE TABLE `road_target_step_combination_all` (
  `tsrc_id` int(11) NOT NULL,
  `step_id` int(11) NOT NULL,
  `target_id` int(11) NOT NULL,
  `step_type` int(11) NOT NULL COMMENT '1= digging, 2=cutting, 3=filling (material)',
  `step_seq_number` int(11) NOT NULL,
  `completion_in_days` int(11) NOT NULL,
  `refer_cycle_id` int(11) NOT NULL COMMENT 'road_cycle_detail_all',
  `cycle_completion_days` int(11) NOT NULL,
  `material_id` int(11) NOT NULL COMMENT 'reference from road_layers_header_all'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `second_sub_folder_details_all`
--

CREATE TABLE `second_sub_folder_details_all` (
  `sec_id` varchar(20) NOT NULL,
  `sec_folder_name` longtext DEFAULT NULL,
  `first_id` varchar(20) DEFAULT NULL,
  `pac_id` varchar(20) DEFAULT NULL,
  `tdoc_id` varchar(20) DEFAULT NULL,
  `t_pdc_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `state_header_all`
--

CREATE TABLE `state_header_all` (
  `st_id` varchar(20) NOT NULL,
  `st_name` varchar(100) NOT NULL,
  `st_geojson_file` longtext DEFAULT NULL,
  `st_inserted_on` datetime NOT NULL,
  `st_status` int(11) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `taluka_header_all`
--

CREATE TABLE `taluka_header_all` (
  `tal_id` varchar(20) NOT NULL,
  `tal_name` varchar(100) NOT NULL,
  `tal_geojson_file` longtext DEFAULT NULL,
  `tal_inserted_on` datetime NOT NULL,
  `tal_status` int(11) NOT NULL,
  `dist_id` varchar(20) NOT NULL,
  `st_id` varchar(20) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `tender_doc_header_all`
--

CREATE TABLE `tender_doc_header_all` (
  `tdoc_id` varchar(20) NOT NULL,
  `tdoc_name` longtext DEFAULT NULL,
  `tdoc_type` int(11) NOT NULL,
  `tdoc_status` int(11) NOT NULL,
  `tdoc_inserted_on` datetime DEFAULT NULL,
  `tdoc_permited_extension` longtext DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `tender_package_doc_combination_all`
--

CREATE TABLE `tender_package_doc_combination_all` (
  `t_pdc_id` varchar(20) NOT NULL,
  `inserted_on` datetime NOT NULL,
  `valid_till` datetime DEFAULT NULL,
  `line_no` int(11) DEFAULT NULL,
  `ph_id` varchar(20) DEFAULT NULL,
  `tdoc_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `tender_package_doc_data_all`
--

CREATE TABLE `tender_package_doc_data_all` (
  `pdd_id` varchar(20) NOT NULL,
  `file_name_1` varchar(50) DEFAULT NULL,
  `file_name_40` varchar(50) DEFAULT NULL,
  `coh_id` varchar(20) DEFAULT NULL,
  `first_id_id` varchar(20) DEFAULT NULL,
  `pac_id` varchar(20) DEFAULT NULL,
  `second_id_id` varchar(20) DEFAULT NULL,
  `t_pdc_id` varchar(20) DEFAULT NULL,
  `tdoc_id` varchar(20) DEFAULT NULL,
  `third_id_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `third_sub_folder_details_all`
--

CREATE TABLE `third_sub_folder_details_all` (
  `third_id` varchar(20) NOT NULL,
  `third_folder_name` varchar(100) NOT NULL,
  `first_id` varchar(20) NOT NULL,
  `pac_id` varchar(20) DEFAULT NULL,
  `sec_id` varchar(20) NOT NULL,
  `t_pdc_id` varchar(20) NOT NULL,
  `tdoc_id` varchar(20) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `three_d_files_details_all`
--

CREATE TABLE `three_d_files_details_all` (
  `tdfd_id` varchar(20) NOT NULL,
  `tdfd_no` varchar(255) DEFAULT NULL,
  `tdfd_start_gps` longtext DEFAULT NULL,
  `tdfd_end_gps` longtext DEFAULT NULL,
  `tdfd_inserted_on` datetime NOT NULL,
  `ph_id` varchar(20) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `unique_id_header_all`
--

CREATE TABLE `unique_id_header_all` (
  `uh_id` int(11) NOT NULL,
  `uh_table_name` varchar(100) NOT NULL,
  `uh_id_for` varchar(50) NOT NULL,
  `uh_prefix` varchar(3) NOT NULL,
  `uh_last_id` varchar(15) NOT NULL,
  `uh_created_on` datetime NOT NULL,
  `uh_modified_on` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Dumping data for table `unique_id_header_all`
--

INSERT INTO `unique_id_header_all` (`uh_id`, `uh_table_name`, `uh_id_for`, `uh_prefix`, `uh_last_id`, `uh_created_on`, `uh_modified_on`) VALUES
(1, 'user_header_all', 'user_header_all_id', 'UHA', 'UHA-00002', '2026-01-28 11:19:32', '2026-01-28 11:24:06');

-- --------------------------------------------------------

--
-- Table structure for table `user_header_all`
--

CREATE TABLE `user_header_all` (
  `id` int(11) NOT NULL COMMENT 'PRIMARY KEY',
  `user_id` varchar(15) DEFAULT NULL,
  `user_full_name` varchar(150) NOT NULL,
  `user_email` varchar(150) NOT NULL,
  `user_username` varchar(150) NOT NULL,
  `user_password` varchar(128) NOT NULL,
  `user_mobile_no` varchar(10) NOT NULL,
  `user_type` int(11) NOT NULL COMMENT '1=platform,2= dept, 3= contractor, 4= sub contractor, 5=mobile tab',
  `status` int(11) NOT NULL COMMENT '1=active, 2= non active',
  `user_inserted_on` datetime NOT NULL,
  `user_deactivated_on` datetime DEFAULT NULL,
  `valid_till` datetime NOT NULL,
  `permitted_operation` int(2) NOT NULL COMMENT '1=web,2=mobile csv',
  `mobile_token` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Dumping data for table `user_header_all`
--

INSERT INTO `user_header_all` (`id`, `user_id`, `user_full_name`, `user_email`, `user_username`, `user_password`, `user_mobile_no`, `user_type`, `status`, `user_inserted_on`, `user_deactivated_on`, `valid_till`, `permitted_operation`, `mobile_token`) VALUES
(1, 'UHA-00001', 'Aniket Pund', 'aniket1607@gamil.com', 'aniket1607', 'pbkdf2_sha256$29000$QFbsOarBjoVJ$ysn6IV9fQtqx9lTOxP0dYtmGt+yHVjo32fskDEQw0W0=', '9423720042', 1, 1, '2026-01-28 11:19:32', NULL, '9999-12-31 23:59:59', 1, ''),
(3, 'UHA-00002', 'Santosh Mechewad', 'santoshmechewad@gmail.com', 'santosh2002', 'pbkdf2_sha256$29000$JVUzRLXwcLDB$diSyChkudPFnuy8CZyv3DtXquCrx6BUuT4edDg8vE20=', '7887765336', 1, 1, '2026-01-28 11:24:06', NULL, '9999-12-31 23:59:59', 1, '');

-- --------------------------------------------------------

--
-- Table structure for table `village_header_all`
--

CREATE TABLE `village_header_all` (
  `vil_id` varchar(20) NOT NULL,
  `vil_name` varchar(100) NOT NULL,
  `vil_inserted_on` datetime NOT NULL,
  `vil_status` int(11) NOT NULL,
  `dist_id` varchar(20) NOT NULL,
  `st_id` varchar(20) NOT NULL,
  `tal_id` varchar(20) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `worksheet_layers_header_all`
--

CREATE TABLE `worksheet_layers_header_all` (
  `id` int(11) NOT NULL COMMENT 'Primary Key',
  `layer_id` varchar(25) NOT NULL,
  `layer_name` varchar(25) NOT NULL COMMENT 'Layer Name - Ex: Design Layer - 1, Material Layer - 1',
  `worksheet_id` int(25) NOT NULL COMMENT 'project_worksheet_header_all',
  `2D` int(1) DEFAULT 0 COMMENT '1=Yes, 0=No',
  `2D_type` int(1) DEFAULT NULL COMMENT '1=Design, 2=Material',
  `3D` tinyint(1) DEFAULT 0 COMMENT '1=Yes, 0=No',
  `3D_type` int(1) DEFAULT NULL COMMENT '1=Road ref layer, 2=Bridge ref layer, 3=Merger layer, 4=Measurement layer',
  `status` tinyint(1) DEFAULT 1 COMMENT '1=In use, 2=Not in use',
  `3D_file` longtext DEFAULT NULL COMMENT '3D file path',
  `from_km` int(11) DEFAULT NULL,
  `from_chainage` int(11) DEFAULT NULL,
  `to_km` int(11) DEFAULT NULL,
  `to_chainage` int(11) DEFAULT NULL,
  `inserted_on` datetime DEFAULT current_timestamp(),
  `layer_road_data` longtext DEFAULT NULL COMMENT 'Road layer related data config data only',
  `layer_bridge_data` longtext DEFAULT NULL COMMENT 'Bridge layer related data config data only',
  `layer_measurement_data` longtext DEFAULT NULL COMMENT 'Measurement layer related data',
  `chainage_interval` int(2) NOT NULL COMMENT 'its a chainage seq like if 5 is enter then no will be ',
  `rejected_on` datetime NOT NULL,
  `inseted_by` varchar(25) NOT NULL COMMENT 'user_id who enter lauer from the desktop',
  `operational_status` int(1) NOT NULL COMMENT '1=under approval, 2= approved, 3= rejected',
  `ph_id` varchar(25) NOT NULL COMMENT 'project id',
  `rejected_by` varchar(25) NOT NULL COMMENT 'web user who rejected this layer',
  `reason_of_rejection` varchar(200) NOT NULL COMMENT 'the reason given by web person and then communicate to the desktop person'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- --------------------------------------------------------

--
-- Table structure for table `work_stage_target_details_all`
--

CREATE TABLE `work_stage_target_details_all` (
  `wst_id` varchar(25) NOT NULL,
  `cws_id` varchar(25) NOT NULL COMMENT 'contract_work_stages_details_all',
  `target_1` varchar(25) DEFAULT NULL COMMENT 'unique number generated',
  `target_1_details` longtext DEFAULT NULL COMMENT 'activities nedded to conduct in target and other details in json format',
  `target_1_days` int(11) DEFAULT NULL COMMENT 'days to complete the activities',
  `target_1_type` int(2) DEFAULT NULL COMMENT '1=Digging,2=Filling,3=Material Laying, 4=Rolling, 5=Base Pipe Fixing, 6=Well Base Construction, 7= Pillar',
  `gap_1_days` int(11) DEFAULT NULL COMMENT 'gap taken after completion of first target before starting the second target',
  `target_2` varchar(25) DEFAULT NULL,
  `target_2_details` longtext DEFAULT NULL,
  `target_2_days` int(11) DEFAULT NULL,
  `target_2_type` varchar(25) DEFAULT NULL,
  `gap_2_days` int(11) DEFAULT NULL,
  `target_3` varchar(25) DEFAULT NULL,
  `target_3_details` longtext DEFAULT NULL,
  `target_3_days` int(11) DEFAULT NULL,
  `target_3_type` varchar(25) DEFAULT NULL,
  `gap_3_days` int(11) DEFAULT NULL,
  `target_4` varchar(25) DEFAULT NULL,
  `target_4_details` longtext DEFAULT NULL,
  `target_4_days` int(11) DEFAULT NULL,
  `target_4_type` varchar(25) DEFAULT NULL,
  `gap_4_days` int(11) DEFAULT NULL,
  `target_5` varchar(25) DEFAULT NULL,
  `target_5_details` longtext DEFAULT NULL,
  `target_5_days` int(11) DEFAULT NULL,
  `target_5_type` varchar(25) DEFAULT NULL,
  `gap_5_days` int(11) DEFAULT NULL,
  `target_6` varchar(25) DEFAULT NULL,
  `target_6_details` longtext DEFAULT NULL,
  `target_6_days` int(11) DEFAULT NULL,
  `target_6_type` varchar(25) DEFAULT NULL,
  `gap_6_days` int(11) DEFAULT NULL,
  `target_7` varchar(25) DEFAULT NULL,
  `target_7_details` longtext DEFAULT NULL,
  `target_7_days` int(11) DEFAULT NULL,
  `target_7_type` varchar(25) DEFAULT NULL,
  `gap_7_days` int(11) DEFAULT NULL,
  `target_8` varchar(25) DEFAULT NULL,
  `target_8_details` longtext DEFAULT NULL,
  `target_8_days` int(11) DEFAULT NULL,
  `target_8_type` varchar(25) DEFAULT NULL,
  `gap_8_days` int(11) DEFAULT NULL,
  `target_9` varchar(25) DEFAULT NULL,
  `target_9_details` longtext DEFAULT NULL,
  `target_9_days` int(11) DEFAULT NULL,
  `target_9_type` varchar(25) DEFAULT NULL,
  `gap_9_days` int(11) DEFAULT NULL,
  `target_10` varchar(25) DEFAULT NULL,
  `target_10_details` longtext DEFAULT NULL,
  `target_10_days` int(11) DEFAULT NULL,
  `target_10_type` varchar(25) DEFAULT NULL,
  `gap_10_days` int(11) DEFAULT NULL,
  `target_11` varchar(25) DEFAULT NULL,
  `target_11_details` longtext DEFAULT NULL,
  `target_11_days` int(11) DEFAULT NULL,
  `target_11_type` varchar(25) DEFAULT NULL,
  `gap_11_days` int(11) DEFAULT NULL,
  `target_12` varchar(25) DEFAULT NULL,
  `target_12_details` longtext DEFAULT NULL,
  `target_12_days` int(11) DEFAULT NULL,
  `target_12_type` varchar(25) DEFAULT NULL,
  `gap_12_days` int(11) DEFAULT NULL,
  `target_13` varchar(25) DEFAULT NULL,
  `target_13_details` longtext DEFAULT NULL,
  `target_13_days` int(11) DEFAULT NULL,
  `target_13_type` varchar(25) DEFAULT NULL,
  `gap_13_days` int(11) DEFAULT NULL,
  `target_14` varchar(25) DEFAULT NULL,
  `target_14_details` longtext DEFAULT NULL,
  `target_14_days` int(11) DEFAULT NULL,
  `target_14_type` varchar(25) DEFAULT NULL,
  `gap_14_days` int(11) DEFAULT NULL,
  `target_15` varchar(25) DEFAULT NULL,
  `target_15_details` longtext DEFAULT NULL,
  `target_15_days` int(11) DEFAULT NULL,
  `target_15_type` varchar(25) DEFAULT NULL,
  `gap_15_days` int(11) DEFAULT NULL,
  `line_no` int(2) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

--
-- Indexes for dumped tables
--

--
-- Indexes for table `bridge_layer_data_details_all`
--
ALTER TABLE `bridge_layer_data_details_all`
  ADD PRIMARY KEY (`lbad_id`);

--
-- Indexes for table `layer_cross_reference_details_all`
--
ALTER TABLE `layer_cross_reference_details_all`
  ADD PRIMARY KEY (`lcrd_id`);

--
-- Indexes for table `project_worksheet_header_all`
--
ALTER TABLE `project_worksheet_header_all`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `road_layers_data_details_all`
--
ALTER TABLE `road_layers_data_details_all`
  ADD PRIMARY KEY (`lrdd_id`);

--
-- Indexes for table `road_step_gap_details_all`
--
ALTER TABLE `road_step_gap_details_all`
  ADD PRIMARY KEY (`rsgd_id`);

--
-- Indexes for table `road_target_header_all`
--
ALTER TABLE `road_target_header_all`
  ADD PRIMARY KEY (`trh_id`);

--
-- Indexes for table `road_target_step_combination_all`
--
ALTER TABLE `road_target_step_combination_all`
  ADD PRIMARY KEY (`tsrc_id`);

--
-- Indexes for table `unique_id_header_all`
--
ALTER TABLE `unique_id_header_all`
  ADD PRIMARY KEY (`uh_id`);

--
-- Indexes for table `user_header_all`
--
ALTER TABLE `user_header_all`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `worksheet_layers_header_all`
--
ALTER TABLE `worksheet_layers_header_all`
  ADD PRIMARY KEY (`id`);

--
-- AUTO_INCREMENT for dumped tables
--

--
-- AUTO_INCREMENT for table `bridge_layer_data_details_all`
--
ALTER TABLE `bridge_layer_data_details_all`
  MODIFY `lbad_id` int(255) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `layer_cross_reference_details_all`
--
ALTER TABLE `layer_cross_reference_details_all`
  MODIFY `lcrd_id` int(25) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `project_worksheet_header_all`
--
ALTER TABLE `project_worksheet_header_all`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `road_layers_data_details_all`
--
ALTER TABLE `road_layers_data_details_all`
  MODIFY `lrdd_id` int(25) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `road_step_gap_details_all`
--
ALTER TABLE `road_step_gap_details_all`
  MODIFY `rsgd_id` int(11) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `road_target_header_all`
--
ALTER TABLE `road_target_header_all`
  MODIFY `trh_id` int(11) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `road_target_step_combination_all`
--
ALTER TABLE `road_target_step_combination_all`
  MODIFY `tsrc_id` int(11) NOT NULL AUTO_INCREMENT;

--
-- AUTO_INCREMENT for table `unique_id_header_all`
--
ALTER TABLE `unique_id_header_all`
  MODIFY `uh_id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;

--
-- AUTO_INCREMENT for table `user_header_all`
--
ALTER TABLE `user_header_all`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT COMMENT 'PRIMARY KEY', AUTO_INCREMENT=4;

--
-- AUTO_INCREMENT for table `worksheet_layers_header_all`
--
ALTER TABLE `worksheet_layers_header_all`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT COMMENT 'Primary Key';
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
