$(document).ready(function() {
    $('select').material_select();

	var ctx = document.getElementById("myChart").getContext('2d');
	var myChart = new Chart(ctx, {
	  type: 'bar',
	  data: {
	    labels: ["6/4", "6/11", "6/18", "6/25", "7/2", "7/6"],
	    datasets: [{
	      label: 'positive',
	      data: [12, 19, 13, 17, 20, 24],
	      backgroundColor: '#ABC233'
	    }, {
	      label: 'negative',
	      data: [1, 3, 5, 5, 2, 3],
	      backgroundColor: "#C4122F"
	    }]
	  }
	});

	$('.deploy-btn').click(function() {
		var table = $('.table-data tr:last').after('<tr><td>Foxconn</td> \
				<td>Labor Audit</td> \
				<td>30 workers</td> \
				<td>$100 left</td> \
				<td>7 days to go</td> \
				<td><a class="material-icons modal-trigger info dash">info_outline</a></td></tr>')
		$('.dash').click(function() {
			$('#modal1').modal('open');
		})
	})
});

/*
Pop Up:
phone number
contact address

data
*/