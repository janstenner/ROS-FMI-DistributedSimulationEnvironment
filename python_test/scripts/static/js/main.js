/** Utility functions START **/
// create unique id
function uuidv4() {
	return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
		var r = Math.random() * 16 | 0, v = c == 'x' ? r : (r & 0x3 | 0x8);
	return v.toString(16);
  });
}

// convert arraybuffer to base64 encoded string
function arrayBufferToBase64(buffer) {
	let binary = '';
	let bytes = new Uint8Array(buffer);
	let len = bytes.byteLength;
	for (let i = 0; i < len; i++) {
		binary += String.fromCharCode(bytes[i]);
	}
	return window.btoa(binary);
}

// create a downloadable text file
function download(filename, text) 
{
	var element = document.createElement('a');
	element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
	element.setAttribute('download', filename);
	element.style.display = 'none';
	document.body.appendChild(element);
	element.click();
	document.body.removeChild(element);
}

// convert JSON format strings to objects
function unescapeandJSON(string)
{
	// accessing text input from html gives escaped strings which breaks the JSON parser
	var replaced_string = string.replace(/\\/g, '');
	return JSON.parse(replaced_string);
}
/** Utility functions END **/

var socket = io();

/**
* info tab
**/
function fill_node_info_board(node_info)
{
	$("#node_info").empty().append($("#node_info_template").html())
	
	var data = JSON.parse(node_info);
	if(data.hasOwnProperty("error"))
	{
		alert(data.error);
		return;
	}
	$("#node_info_id").append(data.node_id);
	$("#node_info_hostname").append(data.hostname);
	$("#node_info_fmus").append(JSON.stringify(data.fmus));
	$("#node_info_connections").append(JSON.stringify(data.connections));
	$("#node_info_ros_input").append(JSON.stringify(data.ros_input));
	$("#node_info_ros_output").append(JSON.stringify(data.ros_output));
}

/**
* graph tab START
**/
// create select options for the output paths
function fill_node_graph_board(output_info)
{
	$("#node_graph").empty().append($("#node_graph_template").html())
	
	var data = JSON.parse(output_info);
	if(data.hasOwnProperty("error"))
	{
		alert(data.error);
		return;
	}
	data.forEach(function(entry){
		$('#graph_topic_selector').append($('<option>').val(entry).text(entry));
	});
}

// start a new graph stream
$(document).on('click', '#start_graph_button', function (event) {
	var topic = $('#graph_topic_selector option:selected').val();
	socket.emit('request_graph_stream', topic , handle_graph_stream_feedback);
});

// stop current graph stream (if any available)
$(document).on('click', '#stop_graph_button', function (event) {
	socket.emit('stop_graph_stream');
});

var lineChart;
var config;
	
// if a graph stream sends data, this function will fill the linechart
// the chart holds up to 200 values and then shifts values
socket.on('graph_data', function(data) {
	if (config.data.labels.length === 200) {
		config.data.labels.shift();
		config.data.datasets[0].data.shift();
	}
	config.data.labels.push(data.time);
	config.data.datasets[0].data.push(data.value);
	lineChart.update();
});

function handle_graph_stream_feedback(response)
{
	
	if(response == true)
	{
		if( lineChart != undefined)
			lineChart.destroy();
		
		config = {
			type: 'line',
			data: {
				labels: [],
				datasets: [{
					label: "Sum of all joint angles",
					backgroundColor: 'rgb(255, 99, 132)',
					borderColor: 'rgb(255, 99, 132)',
					data: [],
					fill: false,
				}],
			},
			options: {
				responsive: true,
				title: {
					display: true,
					text: 'Robot joints angles sum'
				},
				tooltips: {
					mode: 'index',
					intersect: false,
				},
				hover: {
					mode: 'nearest',
					intersect: true
				},
				scales: {
					xAxes: [{
						display: true,
						scaleLabel: {
							display: true,
							labelString: 'Time'
						}
					}],
					yAxes: [{
						display: true,
						scaleLabel: {
							display: true,
							labelString: 'Value'
						}
					}]
				}
			}
		};
        const context = document.getElementById('graph_canvas').getContext('2d');
        lineChart = new Chart(context, config);
	}
}
/**
* graph tab END
**/

/**
* config tab START
**/
// creates a new fmu file entry with unique id
function create_new_fmu_file_entry()
{
	var a = $($("#fmu_file_entry_template").html())
	var unique_id = uuidv4();
	a.find("#myInput").attr("id",unique_id).attr("aria-describedby",unique_id);
	a.find('[for="myInput"]').attr("for",unique_id);
	
	return a;
}

// create new file entry button handler
$(document).on('click', '#new_fmu_entry_button', function (event) {
	$("#fmu_files").append(create_new_fmu_file_entry());
});

// fmu file entry delete button handler
$(document).on('click', '.fmu_file_entry_container .delete-button', function (event) {
	this.closest(".fmu_file_entry_container").remove();
});

// fmu file entry input: filename will be displayed if user selects a file
$(document).on('change', '.custom-file-input', function (event) {
	var fileName = $(this).prop("files")[0].name;
	$(this).next().text(fileName);
});

// send the event as click event to the hidden file input
$(document).on('click', '#load_config_file', function (event) {
	$("#config_file_input").trigger("click");	        
});

// occurs if the user selected a file in the file input dialog
$(document).on('change', '#config_file_input', function (event) {
	// reset the config tab
	$("#node_config").empty().append($("#configuration_template").html())

	var input = event.target;
	var reader = new FileReader();
	
	reader.onload = function(){
		var data = JSON.parse(reader.result);

		$("#config_ros_input").val(data["ros_in"]);
        $("#config_ros_input_types").val(data["ros_in_types"]);
		$("#config_ros_output").val(data["ros_out"]);
		$("#config_connections").val(data["ros_connections"]);
		
		data["ros_fmus"].forEach(function(entry){
			// create new fmu file entry 
			var a = create_new_fmu_file_entry()
			a.find(".fmu_unique_name_input").val(entry[0]);
			a.find(".fmu_inputs_input").val(entry[1]);
			a.find(".fmu_outputs_input").val(entry[2]);
			$("#fmu_files").append(a);
		});
	};
	reader.readAsText(input.files[0]);
});


$(document).on('change', '#amlx_file_input', function (event) {
	// reset the config tab
	$("#node_config").empty().append($("#configuration_template").html())
	//	let uploadedfile = document.getElementById("config_file_input").files[0];
	//	let uploadedfilename = document.getElementById("config_file_input").files[0].name;
	var input = event.target;

	var files = event.target.files[0];
	console.log("the files are");
	console.log(files);

	var formdata = new FormData();
	var data;
	var reader = new FileReader();

	formdata.append("file", files);
	console.log(formdata);
	console.log("connecting through socket");
	// socket.emit('handle_form_test',formdata, handle_upload_feedbacktest);

	$.ajax({
		url: 'handle_form',
		data: formdata,
		type: 'POST',
		processData: false,
		contentType: false,
		success: function (jsondata) {
			//callback when request is finished.
			//in our script.php, we just echoed the posted username, so it alerts whatever username you have input.
			console.log("the json data is");
			console.log(jsondata);
			data = JSON.parse(jsondata);
			$("#config_ros_input").val(data["ros_in"]);
			$("#config_ros_output").val(data["ros_out"]);
			$("#config_connections").val(data["ros_connections"]);
			data["ros_fmus"].forEach(function (entry) {
				// create new fmu file entry
				var a = create_new_fmu_file_entry()
					a.find(".fmu_unique_name_input").val(entry[0]);
				a.find(".fmu_inputs_input").val(entry[1]);
				a.find(".fmu_outputs_input").val(entry[2]);
				$("#fmu_files").append(a);
			});

		}
	});
	reader.readAsText(input.files[0]);
	console.log("ajax requests sent tzhtz");
});

function handle_upload_feedbacktest(response) {
	console.log("inside feedback");
	console.log("the response is");
	console.log(response);
	console.log("afrer parsing  the response the data is");
	var data = JSON.parse(response);
	console.log("the json data is", data);
	if (data.hasOwnProperty("error")) {
		alert(data.error);
	}
	$("#config_ros_input").val(data["ros_in"]);
	$("#config_ros_output").val(data["ros_out"]);
	$("#config_connections").val(data["ros_connections"]);
	console.log("the ros_fmus is", data["ros_fmus"]);
	data["ros_fmus"].forEach(function (entry) {
		// create new fmu file entry
		var a = create_new_fmu_file_entry()
			a.find(".fmu_unique_name_input").val(entry[0]);
		a.find(".fmu_inputs_input").val(entry[1]);
		a.find(".fmu_outputs_input").val(entry[2]);
		$("#fmu_files").append(a);
	});
}

// create a config file and open a download dialog
$(document).on('click', '#save_config_file', function (event) {
	var fmus_config = [];
	
	// from fmu entries we generate objects with the uniquename,inputs and outputs
	// INFO: The config file does NOT store the .fmu file! 
	$(".fmu_file_entry_container").each(function( index, element ) {
		fmus_config.push([
			$(element).find(".fmu_unique_name_input").val(), // unique name
			$(element).find(".fmu_inputs_input").val(), // inputs
			$(element).find(".fmu_outputs_input").val()]) ;// outputs
	});

	var payload={
		"ros_in" : $("#config_ros_input").val(),
        "ros_in_types" : $("#config_ros_input_types").val(),
		"ros_out" : $("#config_ros_output").val(),
		"ros_connections" : $("#config_connections").val(),
		"ros_fmus" : fmus_config  // uniquename, inputs, outputs
	}

	download("Node_Configuration.txt",JSON.stringify(payload));
});

// if the upload was successful, the server also broadcasts the node_configured event.
// therefor we will use this callback only to catch errors that happened during configuration
// -> the node_configured event will handle all necessary steps to update the UIs for connected dashboard clients.
function handle_upload_feedback(data){
	var data = JSON.parse(data);
	if(data.hasOwnProperty("error")){
		alert(data.error);
	}
}

// convert the text inputs to objects and send the payload to the node
$(document).on('click', '#upload_to_node', function (event) {
	var fmu_byte_arrays = []
	
	// create promise which delivers the .fmu file in base64 encoded format
	const readFMUfiles = (inputFile) => {
		const temporaryFileReader = new FileReader();

		return new Promise((resolve, reject) => {
			temporaryFileReader.onerror = () => {
				temporaryFileReader.abort();
				reject(new DOMException("Problem parsing input file."));
			};

			temporaryFileReader.onload = () => {
				resolve([inputFile.name,arrayBufferToBase64(temporaryFileReader.result)]);
			};
			temporaryFileReader.readAsArrayBuffer(inputFile);
		});
	};
	
	// create the promises for each fmu file entry
	$("#fmu_files [type='file']").each(function( index, element ) {
		fmu_byte_arrays.push( readFMUfiles( $(element).prop("files")[0] ));
	});
	
	// if all promises are resolved we generate the payload and send it out
	Promise.all(fmu_byte_arrays).then(values => { 
		var fmus_config = [];
		$(".fmu_file_entry_container").each(function( index, element ) {
			fmus_config.push([
				$($(element).find("[type='file']")[0]).next().text(), // filename
				$(element).find(".fmu_unique_name_input").val(), // unique name
				unescapeandJSON($(element).find(".fmu_inputs_input").val()), // inputs
				unescapeandJSON($(element).find(".fmu_outputs_input").val()) // outputs
			]) ;
		});
		
		// get the node id
		var id = "/"+$("#node_list :checked")[0].id;
        
		if ($("#config_ros_input_types").val()) {
            var payload={
                "ros_in" : unescapeandJSON($("#config_ros_input").val()),
                "ros_in_types" : unescapeandJSON($("#config_ros_input_types").val()),
                "ros_out" : unescapeandJSON($("#config_ros_output").val()),
                "ros_connections" : unescapeandJSON($("#config_connections").val()),
                "ros_fmu_files" : values,
                "ros_fmus" : fmus_config  // filename, uniquename, inputs, outputs
            }
        } else {
            var payload={
                "ros_in" : unescapeandJSON($("#config_ros_input").val()),
                "ros_out" : unescapeandJSON($("#config_ros_output").val()),
                "ros_connections" : unescapeandJSON($("#config_connections").val()),
                "ros_fmu_files" : values,
                "ros_fmus" : fmus_config  // filename, uniquename, inputs, outputs
            }
        }

		socket.emit('set_configurations',id, payload, handle_upload_feedback);
	});
});

/**
* config tab END
**/

socket.on('connect', function() {
	console.log("Trying to connect...");
	socket.emit('dashboard_connect', dashboard_connected);
});

// create a new entry in the node list
function create_node_entry(value)
{
	var a = $($("#node_entry_template").html());
	a.find("#node_entry_name").append(value);
	a.find("input").attr("id",value.substring(1));
	a.appendTo('#node_list');
}

// when dashboard is connected, fill the node list and reset the tab content.
function dashboard_connected(nodes)
{
	console.log("Dashboard connected!");

	$('#myTabContent div').empty().append('<span class="no_node_selected">Please select a node!</span>');
	$('#node_list label').remove();
	
	$.each(nodes, function( index, value ) {
		create_node_entry(value);
	});
}

// changed selection of nodes in the node list
$(document).on('change', 'input:radio[id^="fmu"]', function (event) {
	$('#myTabContent div').empty().append('<span class="no_node_selected">Loading node information...</span>')
	
	// in case the user started a graph and didnt stop it, we send the stop message just to be safe
	socket.emit('stop_graph_stream');

	// request the node information
	socket.emit('get_node_info',"/"+this.id, fill_node_info_board);
	$("#node_config").empty().append($("#configuration_template").html())
	socket.emit('get_nodes_outputs',"/"+this.id, fill_node_graph_board);
});
	
socket.on('node_configured', function(id) 
{
	// we want to inform the user that a node has been configured, if it was the currently selected one
	if($("#node_list :checked")[0] != undefined){
		var current_selected_id = $("#node_list :checked")[0].id;
		if(current_selected_id == id.substring(1))
		{
			alert("Node " + id + " has been configured!");
			
			// reload the info from the node, stop graph streams and update output select 
			socket.emit('stop_graph_stream');
			socket.emit('get_node_info',"/"+$("#node_list :checked")[0].id, fill_node_info_board);
			socket.emit('get_nodes_outputs',"/"+$("#node_list :checked")[0].id, fill_node_graph_board);
		}
	}
});

socket.on('new_node_connected', function(data) {
	console.log("connected node" ,data);
	create_node_entry(data)
});

socket.on('node_disconnected', function(data) {
	console.log("disconnected node" ,data);
	
	// if the node was the currently selected one, inform user with an alert and reset the tabs
	if($("#node_list :checked")[0] != undefined){
		var current_selected_id = $("#node_list :checked")[0].id;
		if(current_selected_id == data.substring(1))
		{
			// also stop potential graph streams
			socket.emit('stop_graph_stream');
			alert("Currently selected node was disconnected!");
			$('#myTabContent div').empty().append('<span class="no_node_selected">Please select a node!</span>');
		}
	}
	// update the node list
	$("#"+data.substring(1)).closest("label").remove();
});
